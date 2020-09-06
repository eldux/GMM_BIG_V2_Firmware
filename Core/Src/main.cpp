/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/**
 * @brief UAVCAN headers
 */
#include <uavcan_stm32/uavcan_stm32.hpp>
#include <uavcan/uavcan.hpp>

#include <uavcan/protocol/param_server.hpp>

#include <msg/MotorStatus.hpp>

#include "as5048a.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_EVENT_SAVE_CONFIG	( 1 << 0 )
#define MAIN_EVENT_BROADCAST    ( 1 << 1 )
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
osTimerId BroadcastTimerHandle;
osStaticTimerDef_t BroadcastTimerControlBlock;
/* USER CODE BEGIN PV */
/* Declare a variable to hold the handle of the created event group. */
EventGroupHandle_t xMainEventGroupHandle;

/* Declare a variable to hold the data associated with the created
event group. */
StaticEventGroup_t xMainEventGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);
void BroadcastTimer_Callback(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Initialization of UAVCAN Parameters structure
 * @note for first time flash board strore initial parameters in the memory and change them while you need them
 */
struct Parameters configuration = {false, 1, 2.0f, false, 7, 1, 0.2f, 20.0f, 1000.0f, 0.01f, 0.0f, 0, 5};

/**
 * @brief NDEBUG for SWO IDE debuging
 */
/*
#ifndef NDEBUG // Set up printf over SWO (use SWV ITM Viewer window)
  // https://github.com/ethanhuanginst/STM32CubeIDE-Workshop-2019/blob/master/hands-on/03_printf/printf_over_swo.md
  #ifdef __GNUC__
    #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #else
    #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #endif // __GNUC__
  */
/**
 * @brief Retargets the C library printf function to the USART.
 * @param None
 * @retval None
 */
/*
  PUTCHAR_PROTOTYPE {
    //HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
      ITM_SendChar((uint32_t)ch);
    return ch;
  }
#endif
*/

/**
 * @brief Retargets the C library printf function to the USART.
 */
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int std::fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#ifdef __cplusplus
 extern "C" {
#endif
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
    /* e.g. write a character to the USART2 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
#ifdef __cplusplus
}
#endif

/**
 * @brief Static thread for UAVCAN.
 * @note check in the main for a new thread call function
 */
#define CAN_THRAD_STACK_SIZE 1024
uint32_t defaultTaskBuffer[ CAN_THRAD_STACK_SIZE ];
osStaticThreadDef_t defaultTaskControlBlock;
#define CAN_STACK_SIZE 1024
uint32_t defaultTaskBuffer1[ CAN_STACK_SIZE ];
osStaticThreadDef_t defaultTaskControlBlock1;
/**
 * @brief AS5048A Magnetic encoder constructor
 * @param - SPI reference
 * @param - CS pin port
 * @param - CS pin number
 * @param - Timer reference for miliseconds
 */
AS5048A angleSensor(&hspi1, GPIOA, GPIO_PIN_4, &htim4);

/**
 * @brief TI DRV8313 and Field Orientation Control method constructor
 * @note Initial motor 11 poles pairs
 * @param - initial pole number
 * @param - Motor PWM timer reference
 * @note  - Hard coded Motor Driver Enable and Disable. Different drivers uses different approach to enable PWM's.
 * @note Motor Control thread declaration
 */
BLDCMotor myBLDC(11, &htim3, &htim4); //motor pole number
void StartMotorTask(void const * argument);

/**
 *	@brief Motor Calibration. Finding number of poles
 *	@note Founded pole number is written into EEPROM for the UAVCAN param usage
 */
osThreadId MotorCalibrateTaskHandle;
void StartMotorCalibrateTask(void const * argument);

/**
 *	@brief EEPROM memory constructor
 */
EEPROM myEEPROM(&hi2c1);

/**
 *	@brief INA226 voltage, current and power measurement constructor
 */
INA226 myINA226(&hi2c1);


/**
 *	@brief UAVCAN pool inicialization with 8192 memory
 */
constexpr unsigned NodePoolSize = 8192;
uavcan_stm32::CanInitHelper<> can;
uavcan::Node<NodePoolSize>& getNode() {
    static uavcan::Node<NodePoolSize> node(can.driver, uavcan_stm32::SystemClock::instance());
    return node;
}

/**
 * @brief Converting float to Uint8_t and back
 */
typedef union
{
 float number;
 uint8_t bytes[4];
} FLOATUNION_t;

FLOATUNION_t num[6];

/**
 * @brief CAN Status SPIN thread
 */
void StartCANStatusTask(void const * argument);


/**
 *	@brief UART receiver callback for testing values acceptance
 *	@note At final version will be removed
 *	@note Accepts as HEX and only integer values. HEX 30 -> 0, HEX 31 -> 1, HEX 2F -> -1 and etc.
 *
 */
/*
uint8_t str[1];
float target_voltage = 4;
uint8_t update_target = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart->Instance==USART1) {
		HAL_UART_Receive_DMA(&huart1, (uint8_t*) str, 1);
		target_voltage = str[0]-'0';
		update_target = 1;
	}
}
*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /**
   * @brief Timer initialization for microsecond tics
   */
  HAL_TIM_Base_Start_IT(&htim4);

  /**
   * @brief CAN ports initialization for the UAVCAN
   * @note It is prohibited to use direct HALCubeMX configurattion
   *
   * @note CAN1 GPIO Configuration
   * @note PB8     ------> CAN1_RX
   * @note PB9     ------> CAN1_TX
   */
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /**
   *  @brief Initial GPIO ports SET high for the SPI usage
   *  @note MPU9250 blocks Encoder usage if it's CS pin is not set to high
   */
  HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MPU_INIT_GPIO_Port, MPU_INIT_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of BroadcastTimer */
  osTimerStaticDef(BroadcastTimer, BroadcastTimer_Callback, &BroadcastTimerControlBlock);
  BroadcastTimerHandle = osTimerCreate(osTimer(BroadcastTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */

  /* Attempt to create the event group. */
  xMainEventGroupHandle = xEventGroupCreateStatic( &xMainEventGroup );

  /* pxEventGroupBuffer was not null so expect the event group to have
  been created? */
  configASSERT( xMainEventGroupHandle );

  /**
   * @brief Static thread for UAVCAN.
   * @note old one left as example
   */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  osThreadStaticDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, CAN_THRAD_STACK_SIZE, defaultTaskBuffer, &defaultTaskControlBlock);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  //TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /**
   * @brief Timer configuration for the microsecond tics
   * @note 72 Prescaler as APB is configured for 72
   * @note MAX_UINT16_NUMBER uses 0XFFFF from AS5048A.h
   * @note some lines are commented from generated by STMCubeMX
   *   //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   *   //sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   *   //if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
   *   //{
   *   //  Error_Handler();
   *   //}
   */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = MAX_UINT16_NUMBER;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  //sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  //sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  //if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  //{
  //  Error_Handler();
  //}
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOT_DRV_RST_GPIO_Port, MOT_DRV_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MPU_INIT_Pin|MPU_CS_Pin|ENCODER_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_LED_Pin|GREEN_LED_Pin|MOT_EN1_Pin|MOT_EN2_Pin 
                          |MOT_EN3_Pin|MOT_DRV_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MOT_DRV_FAULT_Pin */
  GPIO_InitStruct.Pin = MOT_DRV_FAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MOT_DRV_FAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOT_DRV_RST_Pin */
  GPIO_InitStruct.Pin = MOT_DRV_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOT_DRV_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU_INIT_Pin MPU_CS_Pin ENCODER_CS_Pin */
  GPIO_InitStruct.Pin = MPU_INIT_Pin|MPU_CS_Pin|ENCODER_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_LED_Pin GREEN_LED_Pin MOT_EN1_Pin MOT_EN2_Pin 
                           MOT_EN3_Pin MOT_DRV_SLEEP_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|GREEN_LED_Pin|MOT_EN1_Pin|MOT_EN2_Pin 
                          |MOT_EN3_Pin|MOT_DRV_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/**
 *	@brief Motor Calibration. Finding number of poles
 *	@note Founded pole number is written into EEPROM for the UAVCAN param usage
 *	@note gives wrong value if crosses encoder 0 position. everything works while calculations between 0 and 2_PI
 */
void StartMotorCalibrateTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/**
	 * @brief power supply voltage
	 * @note default 12V
	 */
	myBLDC.voltage_power_supply = 12;

	/**
	 * @brief Set FOC loop to be used
	 * @param voltage, velocity, angle
	 */
	myBLDC.controller = ControlType::voltage;
	/**
	 * @brief Set FOC Mathematical algoritm
	 * @note SinePWM or SpaceVectorPWM
	 */
	myBLDC.foc_modulation = FOCModulationType::SinePWM;

	/**
	 * @brief Enable AS5048A Encoder drivers
	 */
	angleSensor.init();

	/**
	 * @brieflink the motor to the sensor
	 * @param Encoder constructor name
	 */
	myBLDC.linkEncoder(&angleSensor);

	/**
	 * @brief Enable DRV8313 and FOC drivers
	 */
	myBLDC.init();

	// pole pairs calculation routine
	//if (DEBUG_MOTOR == 1) printf("Motor pole pair number estimation example\n");
	//if (DEBUG_MOTOR == 1) printf("---------------------------------------------\n");

	float pp_search_voltage = 4; // maximum power_supply_voltage/2

	LOOP:	//not so nice approach. Should be changed to while loop but I am lazy to initialize a pp variable to check condition
	float pp_search_angle = 6*M_PI; // search electrical angle to turn

	// move motor to the electrical angle 0
	myBLDC.setPhaseVoltage(pp_search_voltage,0);
	osDelay(1000);

	// read the sensor angle
	float angle_begin = angleSensor.getAngleInRad();
	osDelay(50);

	// move the motor slowly to the electrical angle pp_search_angle
	float motor_angle = 0;
	while(motor_angle <= pp_search_angle){
		motor_angle += 0.01;
		myBLDC.setPhaseVoltage(pp_search_voltage, motor_angle);
	}
	osDelay(1000);
	// read the sensor value for 180
	float angle_end = angleSensor.getAngleInRad();
	osDelay(50);
	// turn off the motor
	myBLDC.setPhaseVoltage(0,0);
	osDelay(1000);

	// calculate the pole pair number
	uint8_t pp = round((pp_search_angle)/(angle_end-angle_begin));
	configuration.PoleNumber = pp;


	//if (DEBUG_MOTOR == 1) printf("Estimated pole pairs number is: %d\n", pp);
	//if (DEBUG_MOTOR == 1) printf("Electrical angle / Encoder angle = Pole pairs %f/%f=%f\n", pp_search_angle*180/M_PI, (angle_end-angle_begin)*180/M_PI, (pp_search_angle)/(angle_end-angle_begin));

	// a bit of debugging the result
	if(pp <= 0 ){
		//if (DEBUG_MOTOR == 1) printf("Pole pair number cannot be negative\n");
		//if (DEBUG_MOTOR == 1) printf(" - Try changing the search_voltage value or motor/sensor configuration.\n");
		goto LOOP; // try second time and it will work :)
	}else if(pp > 30){
		//if (DEBUG_MOTOR == 1) printf("Pole pair number very high, possible error.\n");
		goto LOOP; // try second time and it will work :)
	}

	/* Infinite loop */
	for(;;) {

		osDelay(1);
		osThreadTerminate(MotorCalibrateTaskHandle);
	}
}

/**
  * @brief Motor cCntrol Thread
  * @param None
  * @retval None
  */
void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	/**
	 * @briefpower supply voltage
	 * @note default 12V
	 */
	myBLDC.voltage_power_supply = 12;

	/**
	 * @brief Enable AS5048A Encoder drivers
	 */
	angleSensor.init();

	/**
	 * @brieflink the motor to the sensor
	 * @param Encoder constructor name
	 */
	myBLDC.linkEncoder(&angleSensor);

	/**
	 * @brief Set FOC Mathematical algoritm
	 * @note SinePWM or SpaceVectorPWM
	 */
	if (configuration.FOCModulation == 0)
		myBLDC.foc_modulation = FOCModulationType::SinePWM;
	if (configuration.FOCModulation == 1)
		myBLDC.foc_modulation = FOCModulationType::SpaceVectorPWM;

	/**
	 * @brief Enable DRV8313 and FOC drivers
	 */
	myBLDC.init();

	/**
	 * @brief align sensor and start FOC
	 */
	myBLDC.initFOC();

	//if (DEBUG_MOTOR == 1) printf("Motor ready.\n");
	//osDelay(10);
	//if (DEBUG_MOTOR == 1) printf("Set the target voltage using serial terminal: \n");
	osDelay(1000);

	/**
	 * @brief fixed value for motor voltage, velocity or angle
	 */
	//float target_voltage = -3;

	/**
	 * @brief Start UART callback for first time to accept motor control parameters
	 * @note later Such receiver will be removed
	 */
	//HAL_UART_Receive_DMA(&huart1, (uint8_t *)str, 1);


	/**
	 * @brief PI controller configuration based on the control type
	 * @note velocity PI controller parameters
	 */
	myBLDC.PI_velocity.P = 0.2f;
	myBLDC.PI_velocity.I = 20.0f;
	myBLDC.PI_velocity.voltage_limit = 6; //default voltage_power_supply/2
	myBLDC.PI_velocity.voltage_ramp = 1000.0f;// jerk control using voltage voltage ramp

	/**
	 * @brief velocity low pass filtering
	 * @note the lower the less filtered
	 */
	myBLDC.LPF_velocity.Tf = 0.01f;

	/**
	 * @brief Primary motor position
	 * @note 0.017 rad is 1 degree. It is some error range.
	 */
	//while(angleSensor.getAngleInRad() > configuration.Angle+0.017 || angleSensor.getAngleInRad() < configuration.Angle-0.017)
	//{

		//myBLDC.controller = ControlType::angle;
		//myBLDC.loopFOC();
		//myBLDC.move(configuration.Angle);
	//}

	/* Infinite loop */
	for(;;) {

		if (configuration.PowerOn == true) {
			/**
			 * @brief Set FOC loop to be used
			 * @param voltage, velocity, angle
			 */

			if (configuration.ControlType == 0)
				myBLDC.controller = ControlType::voltage;
			if (configuration.ControlType == 1)
				myBLDC.controller = ControlType::velocity;
			if (configuration.ControlType == 2)
				myBLDC.controller = ControlType::angle;

			/**
			 * @brief print AS5048A Encoder data
			 */
			//#if (DEBUG_ENCODER == 1)
				//float rad = angleSensor.getAngleInRad();
				//float vel = angleSensor.getVelocity();
				//printf("Current Angle: %f rad\n", rad);
				//printf("Velocity: %f rad/s\n", vel);
			//#endif

			/**
			 * @brief iterative state calculation calculating angle
			 * @brief and setting FOC phase voltage
			 * @brief the faster you run this function the better
			 * @brief the best would be to be in ~10kHz range
			 */
			myBLDC.loopFOC();

			/**
			 * @brief iterative function setting the outer loop target
			 * @brief velocity, position or voltage
			 * @brief this function can be run at much lower frequency than loopFOC function
			 * @brief it can go as low as ~50Hz
			 * @brief the best would be to be in ~10kHz range
			 */
			/*
			if (update_target)
			{
			  update_target = 0;
			  if (DEBUG_MOTOR == 1) printf ("Voltage: %f\n", target_voltage);
			}
			 */

			myBLDC.move(configuration.ControlValue);
		}
	}
}

void StartCANStatusTask(void const * argument){

	for(;;) {

		/*
		 * @brief Health control of the UAVCAN
		 * @param Integer value from timeout in milisoconds
		 */
		const int res = getNode().spin(uavcan::MonotonicDuration::fromMSec(500));
		if (res < 0) {
			//if (DEBUG_MAIN == 1) printf("UAVCAN spin fail\r\n");
			Error_Handler();
		}

	}
}


/* BroadcastTimer_Callback function */
void BroadcastTimer_Callback(void const * argument)
{
  /* USER CODE BEGIN BroadcastTimer_Callback */
  BaseType_t xHigherPriorityTaskWoken;
  xEventGroupSetBitsFromISR(xMainEventGroupHandle, MAIN_EVENT_BROADCAST, &xHigherPriorityTaskWoken);/* The bits being set. */
  /* USER CODE END BroadcastTimer_Callback */
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */

	/**
	 * @brief CAN Speed value
	 */
	uint32_t value = 1000000;

	/*
	 * @brief Enable EEPROM
	 */
	myEEPROM.init();

	/**
	 * @brief Init I2C and INA226 drivers
	 * @param None
	 * @retval None
	 * @note Default INA226 I2C address is 0x40
	 */
	myINA226.init();

	/**
	 * @brief Configure INA226
	 * @param avaraging - fixed values to use from 1 to 1024
	 * @param Bus_Conversion_Time - fixed values to use from 140us to 8244us
	 * @param Shunt_Conversion_Time - fixed values to use from 140us to 8244us
	 * @param Mode - fixed values check library enumerators
	 * @retval None
	 */
	myINA226.configure(INA226_AVERAGES_1, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);

	/**
	 * @brief Calibrate INA226. Rshunt = 0.01 ohm, Max excepted current = 4A
	 * @param Resistor - Shunt resistor value anssembled on the PCB. From Electrical circuit.
	 * @param MAX_Current - MAX current which could flow through Shunt resistor. From Electrical circuit.
	 * @retval None
	 */
	myINA226.calibrate(0.047, 3.5);


	/*
	 * @brief print configuration
	 */
	//myINA226.checkConfig();

	/**
	 * @brief CAN parameters initilisation for UAVCAN
	 * @param CAN speed value
	 * @retval None
	 */
	can.init(value);

	/**
	 * @brief Read data from EEPROM
	 * @param pos   - array or variable where readed data will be pushed
	 * @param start - start address in the momory in HEX format
	 * @param size  - Size of data to read sequentally
	 */

	uint8_t pos[31]={0};
	myEEPROM.readData(pos, 0x04, 31);

	configuration.PowerOn = pos[0];
	configuration.ControlType = pos[1];
	num[0].bytes[0] = pos[2];
	num[0].bytes[1] = pos[3];
	num[0].bytes[2] = pos[4];
	num[0].bytes[3] = pos[5];
	configuration.ControlValue = num[0].number;
	configuration.Calibrate = pos[6];
	configuration.PoleNumber = pos[7];
	configuration.FOCModulation = pos[8];
	num[1].bytes[0] = pos[9];
	num[1].bytes[1] = pos[10];
	num[1].bytes[2] = pos[11];
	num[1].bytes[3] = pos[12];
	configuration.VEL_P = num[1].number ;
	num[2].bytes[0] = pos[13];
	num[2].bytes[1] = pos[14];
	num[2].bytes[2] = pos[15];
	num[2].bytes[3] = pos[16];
	configuration.VEL_I = num[2].number;
	num[3].bytes[0] = pos[17];
	num[3].bytes[1] = pos[18];
	num[3].bytes[2] = pos[19];
	num[3].bytes[3] = pos[20] ;
	configuration.VEL_U_RAMP = num[3].number;
	num[4].bytes[0] = pos[21];
	num[4].bytes[1] = pos[22];
	num[4].bytes[2] = pos[23];
	num[4].bytes[3] = pos[24];
	configuration.VEL_FILTER_Tf = num[4].number;
	num[5].bytes[0] = pos[25];
	num[5].bytes[1] = pos[26];
	num[5].bytes[2] = pos[27];
	num[5].bytes[3] = pos[28];
	configuration.Angle = num[5].number;
	configuration.Orientation = pos[29];
	configuration.NodeID = pos[30];

	/**
	 * @brief Set Board name
	 * @param Name as Char symbols
	 * @retval None
	 */
	getNode().setName("GMM_BIG_V2");

	uavcan::protocol::HardwareVersion hw_ver;
    hw_ver.major = 2;
    hw_ver.minor = 0;
    getNode().setHardwareVersion(hw_ver);
    uavcan::protocol::SoftwareVersion sw_ver;
    sw_ver.major = 2;
    sw_ver.minor = 0;
    getNode().setSoftwareVersion(sw_ver);

	/**
	 * @brief Set Board ID number
	 * @param Integer value from 0 up to 255
	 * @retval None
	 */

	if (configuration.NodeID < 0 || configuration.NodeID > 255) {
		getNode().setNodeID(5);
	} else {
			getNode().setNodeID(configuration.NodeID);
	}


	/**
	 * @brief Start UAVCAN protocol
	 * @retval None
	 * @note stops board if fails
	 */
	if (getNode().start() < 0) {
		//if (DEBUG_MAIN == 1) printf("UAVCAN start fail\r\n");
		while (1);
	}


    /**
     * Now, we need to define some glue logic between the server (below) and our configuration storage (above).
     * This is done via the interface uavcan::IParamManager.
     */

    class : public uavcan::IParamManager {

    	void getParamNameByIndex(Index index, Name& out_name) const override {
    		if (index == 0) { out_name = "Power On/Off"; }
            if (index == 1) { out_name = "Control Type"; }
            if (index == 2) { out_name = "Control Value"; }
            if (index == 3) { out_name = "Calibrate On/Off"; }
       		if (index == 4) { out_name = "Pole Number"; }
       		if (index == 5) { out_name = "FOC Modulation"; }
            if (index == 6) { out_name = "Velocity PID P"; }
            if (index == 7) { out_name = "Velocity PID I"; }
           	if (index == 8) { out_name = "Velocity U Ramp"; }
            if (index == 9) { out_name = "Velocity Filter Tf"; }
            if (index == 10) { out_name = "Primary angle"; }
            if (index == 11) { out_name = "Motor Pitch/Roll/Yaw"; }
            if (index == 12) { out_name = "Node ID"; }
    	}

    	void assignParamValue(const Name& name, const Value& value) override {
    		if (name == "Power On/Off") {
    			if (value.is(uavcan::protocol::param::Value::Tag::boolean_value))
    				configuration.PowerOn = *value.as<uavcan::protocol::param::Value::Tag::boolean_value>();
        	} else if (name == "Control Type") {
        		if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
        			configuration.ControlType = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();
        	} else if (name == "Control Value") {
        		if (value.is(uavcan::protocol::param::Value::Tag::real_value))
        			configuration.ControlValue = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
        	} else if (name == "Calibrate On/Off") {
        		if (value.is(uavcan::protocol::param::Value::Tag::boolean_value))
        			configuration.Calibrate = *value.as<uavcan::protocol::param::Value::Tag::boolean_value>();
        	} else if (name == "Pole Number") {
        		if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
        			configuration.PoleNumber = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();
        	} else if (name == "FOC Modulation") {
        		if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
        			configuration.FOCModulation = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();
        	} else if (name == "Velocity PID P") {
        		if (value.is(uavcan::protocol::param::Value::Tag::real_value))
        			configuration.VEL_P = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
        	} else if (name == "Velocity PID I") {
                if (value.is(uavcan::protocol::param::Value::Tag::real_value))
                	configuration.VEL_I = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
        	} else if (name == "Velocity U Ramp") {
                if (value.is(uavcan::protocol::param::Value::Tag::real_value))
                	configuration.VEL_U_RAMP = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
        	} else if (name == "Velocity Filter Tf") {
                if (value.is(uavcan::protocol::param::Value::Tag::real_value))
                	configuration.VEL_FILTER_Tf = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
        	} else if (name == "Primary angle") {
                if (value.is(uavcan::protocol::param::Value::Tag::real_value))
                	configuration.Angle = *value.as<uavcan::protocol::param::Value::Tag::real_value>();
    		} else if (name == "Motor Pitch/Roll/Yaw") {
    			if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
    				configuration.Orientation = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();
    		} else if (name == "Node ID") {
    			if (value.is(uavcan::protocol::param::Value::Tag::integer_value))
    				configuration.NodeID = *value.as<uavcan::protocol::param::Value::Tag::integer_value>();
        	} //else
        		//if (DEBUG_MAIN == 1) printf("Can't assign parameter. Check if type is correct\r\n");
        }

        void readParamValue(const Name& name, Value& out_value) const override {
        	if (name == "Power On/Off")
        		out_value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = configuration.PowerOn;
        	else if (name == "Control Type")
        		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = configuration.ControlType;
        	else if (name == "Control Value")
        		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.ControlValue;
        	else if (name == "Calibrate On/Off")
            	out_value.to<uavcan::protocol::param::Value::Tag::boolean_value>() = configuration.Calibrate;
          	else if (name == "Pole Number")
          		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = configuration.PoleNumber;
          	else if (name == "FOC Modulation")
          		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = configuration.FOCModulation;
          	else if (name == "Velocity PID P")
          		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.VEL_P;
          	else if (name == "Velocity PID I")
          		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.VEL_I;
          	else if (name == "Velocity U Ramp")
          		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.VEL_U_RAMP;
          	else if (name == "Velocity Filter Tf")
          		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.VEL_FILTER_Tf;
          	else if (name == "Primary angle")
          		out_value.to<uavcan::protocol::param::Value::Tag::real_value>() = configuration.Angle;
          	else if (name == "Motor Pitch/Roll/Yaw")
          		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = configuration.Orientation;
          	else if (name == "Node ID")
          		out_value.to<uavcan::protocol::param::Value::Tag::integer_value>() = configuration.NodeID;
        	//else
        		//if (DEBUG_MAIN == 1) printf("Can't read parameter. Check if type is correct\r\n");
        }

        int saveAllParams() override {
        	//if (DEBUG_MAIN == 1) printf("Save - this implementation does not require any action\r\n");

        	xEventGroupSetBits(xMainEventGroupHandle, MAIN_EVENT_SAVE_CONFIG);/* The bits being set. */
        	osDelay(10);
        	return 0;     // Zero means that everything is fine.
        }

        int eraseAllParams() override {
        	//if (DEBUG_MAIN == 1) printf("Erase - all params reset to default values\r\n");
        	configuration = Parameters();
        	return 0;
        }

/*
        void readParamDefaultMaxMin(const Name& name, Value& out_def, NumericValue& out_max, NumericValue& out_min) const override {
        	if (name == "Control Type") {
        		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() = Parameters().ControlType;
        		out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 2;
        		out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
        	} else if (name == "Pole Number") {
        		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() = Parameters().PoleNumber;
        		out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 30;
        		out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
        	} else if (name == "FOC Modulation") {
        		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() = Parameters().FOCModulation;
        		out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 1;
        		out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
        	} else if (name == "Primary angle") {
        		out_def.to<uavcan::protocol::param::Value::Tag::real_value>() = Parameters().Angle;
        		out_max.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = 6.28f;
        		out_min.to<uavcan::protocol::param::NumericValue::Tag::real_value>() = -6.28f;
        	} else if (name == "Motor Pitch/Roll/Yaw") {
        		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() = Parameters().Orientation;
        		out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 2;
        		out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
        	} else if (name == "Node ID") {
        		out_def.to<uavcan::protocol::param::Value::Tag::integer_value>() = Parameters().NodeID;
        		out_max.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 255;
        		out_min.to<uavcan::protocol::param::NumericValue::Tag::integer_value>() = 0;
        	//} else if (name == "text") {
        		//out_def.to<uavcan::protocol::param::Value::Tag::string_value>() = Parametrai().text.c_str();
        		//DEBUG_Printf("Limits for 'text' are not defined\r\n");
        	} else {
        		//if (DEBUG_MAIN == 1) printf("Can't read the limits for parameter\r\n");
        	}
        }
        */

    } param_manager;

    /**
     * Initializing the configuration server.
     * A pointer to the glue logic object is passed to the method start().
     */
    uavcan::ParamServer server(getNode());

    const int server_start_res = server.start(&param_manager);
    if (server_start_res < 0) {
    	//if (DEBUG_MAIN == 1) printf("Failed to start ParamServer\r\n");
    }

    /*
     * Many embedded applications require a restart before the new configuration settings can
     * be applied, so it is highly recommended to also support the remote restart service.
     */

    class : public uavcan::IRestartRequestHandler {
    	bool handleRestartRequest(uavcan::NodeID request_source) override {
    		HAL_NVIC_SystemReset(); //Ši eilutė leido realiai restartuoti
    		return true;
        }
    } restart_request_handler;


    /**
     * Installing the restart request handler.
     */
    getNode().setRestartRequestHandler(&restart_request_handler); // Done.

	//Motor status publisher
    uavcan::Publisher<msg::MotorStatus> mot_status(getNode());
	mot_status.init();
	msg::MotorStatus mot_status_msg;

	/**
	 * @brief Start UAVCAN normal operation
	 * @retval None
	 * @note Other modes are automatically set by protocol
	 */
	getNode().setModeOperational();


	osThreadStaticDef(CANTask, StartCANStatusTask, osPriorityNormal, 0, CAN_STACK_SIZE, defaultTaskBuffer1, &defaultTaskControlBlock1);
	defaultTaskHandle = osThreadCreate(osThread(CANTask), NULL);

	/**
	 * @brief if first time or calibration needed check
	 */
	//bool Calib = false;
	if (configuration.Calibrate == true) {
		osThreadDef(MotorCalibrateTask, StartMotorCalibrateTask, osPriorityNormal, 0, 128);
		MotorCalibrateTaskHandle = osThreadCreate(osThread(MotorCalibrateTask), NULL);
		configuration.Calibrate  = false;
	} else {
		/**
		 * @brief Start Motor, Encoder and FOC task
		 */
		osThreadDef(MotorTask, StartMotorTask, osPriorityNormal, 0, 128);
		defaultTaskHandle = osThreadCreate(osThread(MotorTask), NULL);
	}

	/**
	 * @brief Enable AS5048A Encoder drivers
	 */
	angleSensor.init();

	/**
	 * @brief Start timer
	*/
	osTimerStart(BroadcastTimerHandle, 50);

	/* Infinite loop */
	for(;;)
	{
		EventBits_t main_events = xEventGroupGetBits( xMainEventGroupHandle );
		if (main_events & MAIN_EVENT_SAVE_CONFIG)
		{
			xEventGroupClearBits(xMainEventGroupHandle, MAIN_EVENT_SAVE_CONFIG );
			uint8_t pos[31]={0};
			pos[0] = configuration.PowerOn;
			pos[1] = configuration.ControlType;
			num[0].number = configuration.ControlValue;
			pos[2] =  num[0].bytes[0];
			pos[3] =  num[0].bytes[1];
			pos[4] =  num[0].bytes[2];
			pos[5] =  num[0].bytes[3];
			pos[6] = configuration.Calibrate;
			pos[7] = configuration.PoleNumber;
			pos[8] = configuration.FOCModulation;
			num[1].number = configuration.VEL_P;
			pos[9] =  num[1].bytes[0];
			pos[10] =  num[1].bytes[1];
			pos[11] =  num[1].bytes[2];
			pos[12] =  num[1].bytes[3];
			num[2].number = configuration.VEL_I;
			pos[13] =  num[2].bytes[0];
			pos[14] =  num[2].bytes[1];
			pos[15] =  num[2].bytes[2];
			pos[16] =  num[2].bytes[3];
			num[3].number = configuration.VEL_U_RAMP;
			pos[17] =  num[3].bytes[0];
			pos[18] =  num[3].bytes[1];
			pos[19] =  num[3].bytes[2];
			pos[20] =  num[3].bytes[3];
			num[4].number = configuration.VEL_FILTER_Tf;
			pos[21] =  num[4].bytes[0];
			pos[22] =  num[4].bytes[1];
			pos[23] =  num[4].bytes[2];
			pos[24] =  num[4].bytes[3];
			num[5].number = configuration.Angle;
			pos[25] =  num[5].bytes[0];
			pos[26] =  num[5].bytes[1];
			pos[27] =  num[5].bytes[2];
			pos[28] =  num[5].bytes[3];
			pos[29] = configuration.Orientation;
			pos[30] = configuration.NodeID;

			myEEPROM.writeData(pos, 0x04, 31);
			osDelay(10);
		}
		if (main_events & MAIN_EVENT_BROADCAST)
		{
			xEventGroupClearBits(xMainEventGroupHandle, MAIN_EVENT_BROADCAST );
			mot_status_msg.axis_id = configuration.Orientation;
			mot_status_msg.motor_pos_rad_raw = myBLDC.shaft_angle_sp;
			mot_status_msg.motor_pos_rad = myBLDC.shaft_angle;
			mot_status_msg.bus_voltage =  myINA226.readBusVoltage();
			mot_status_msg.bus_power = myINA226.readBusPower();
			mot_status_msg.shunt_voltage = myINA226.readShuntVoltage();
			mot_status_msg.shunt_current = myINA226.readShuntCurrent();
			mot_status.broadcast(mot_status_msg);
		}
	    osDelay(1);
	}
  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	/**
	 * @brief Critical error handling
	 * @retval 100ms blinking RED LED on the board
	 */
	for(;;)
	{
		HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
		osDelay(100);
	}

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
