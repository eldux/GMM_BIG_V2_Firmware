/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"

#include "arm_math.h"

#include "stdio.h"
#include "stdbool.h"
#include "malloc.h"
#include "stdarg.h"
#include "timers.h"
#include "string.h"

#include "as5048a.h"
#include "eeprom.h"
#include "ina226.h"
#include "foc.h"

#define DEBUG_MAIN 0
#define DEBUG_EEPROM 0
//#define DEBUG_ENCODER 0
//#define DEBUG_MOTOR 0

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOT_DRV_FAULT_Pin GPIO_PIN_13
#define MOT_DRV_FAULT_GPIO_Port GPIOC
#define MOT_DRV_RST_Pin GPIO_PIN_14
#define MOT_DRV_RST_GPIO_Port GPIOC
#define MPU_INIT_Pin GPIO_PIN_2
#define MPU_INIT_GPIO_Port GPIOA
#define MPU_CS_Pin GPIO_PIN_3
#define MPU_CS_GPIO_Port GPIOA
#define ENCODER_CS_Pin GPIO_PIN_4
#define ENCODER_CS_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_1
#define RED_LED_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_2
#define GREEN_LED_GPIO_Port GPIOB
#define MOT_EN1_Pin GPIO_PIN_10
#define MOT_EN1_GPIO_Port GPIOB
#define MOT_EN2_Pin GPIO_PIN_11
#define MOT_EN2_GPIO_Port GPIOB
#define MOT_EN3_Pin GPIO_PIN_12
#define MOT_EN3_GPIO_Port GPIOB
#define MOT_DRV_SLEEP_Pin GPIO_PIN_13
#define MOT_DRV_SLEEP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
/**
 * This would be our configuration storage.
 */
/**
 * UAVCAN params to control motor
 *
 * Motor Disable/Enable (for new motor default disable) True/False bool -> uint8_t 0/1
 * Motor Control (Voltage/Velocity/Angle) -> uint8_t 0/1/2
 * Motor Motor Contorol value -> float number (Voltage or rad/s or rad)
 * Motor Calibrate True/False bool -> uint8_t 0/1
 * Motor pole number integer -> uint8_t not more than 30
 * Motor FOC Modulation integer -> uint8_t 0/1 (Sin PWM ir Space Vector)
 * Motor PI values ->#define DEF_PI_VEL_P 0.5
					 #define DEF_PI_VEL_I 10
					 #define DEF_PI_VEL_U_RAMP 300
					 #define DEF_VEL_FILTER_Tf 0.005
 * Motor Default angle(orientation) -> float number (rad to direct gimbal at starting point or direction)
 * Motor rotation direction (clockwise or counterclock) -> uint8_t 0/1
 * Motor orientation (Pitch, Yaw, Roll) -> uint8_t 0/1/2
 * UAVCAN Node ID-> uint8_t value up to 255
 * INA226 Enable True/False bool -> uint8_t 0/1
 */
/*
static struct Parameters {
	bool PowerOn = false;
	uint8_t ControlType = 1;
	float ControlValue = 2.0f;
	bool Calibrate = false;
	uint8_t PoleNumber = 7;
	uint8_t FOCModulation = 1;
	float VEL_P = 0.5f;
	float VEL_I = 10.0f;
	float VEL_U_RAMP = 300.0f;
	float VEL_FILTER_Tf = 0.005f;
	float Angle = 0.0f;
	uint8_t Orientation = 0;
	uint8_t NodeID = 5;
	bool EnableINA226 = false;
} configuration;
*/

struct Parameters {
	bool PowerOn;
	uint8_t ControlType;
	float ControlValue;
	bool Calibrate;
	uint8_t PoleNumber;
	uint8_t FOCModulation;
	float VEL_P;
	float VEL_I;
	float VEL_U_RAMP;
	float VEL_FILTER_Tf;
	float Angle;
	uint8_t Orientation;
	uint8_t NodeID;
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
