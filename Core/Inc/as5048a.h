/**
  ******************************************************************************
  * @file           : AS5048A.h
  * @brief          : Header for as5048a.c file.
  *                   This file contains the common defines of the application.
  * @version		: 0.1
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 Raimondas Pomarnacki.
  * All rights reserved.</center></h2>
  *
  * This software component is not licensed,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AS5048A_H
#define __AS5048A_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/* Private includes ----------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include <math.h>
#include "stdio.h"


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

// utility defines
#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI_2 1.57079632679
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038
#define MAX_UINT16_NUMBER 0xFFFF

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#ifdef __cplusplus

class AS5048A{

	uint8_t errorFlag = 0;
	uint16_t _cs;
	uint16_t cs;
	GPIO_TypeDef* _ps;
	SPI_HandleTypeDef* _spi;
	TIM_HandleTypeDef* _tim;
	uint8_t dout;
	uint8_t din;
	uint8_t clk;
	uint16_t position;
	uint16_t transaction(uint16_t data);
	
	public:

	/**
	 *	Constructor
	 */
	AS5048A(SPI_HandleTypeDef *hspi, GPIO_TypeDef* arg_ps, uint16_t arg_cs);
	
	/**
	 * Initialiser
	 * Sets up the SPI interface
	 */
	void init();

	/**
	 * Closes the SPI connection
	 */
	void close();
	
	/**
	 * Open the SPI connection
	 */
	void open();
	
	/*
	 * Read a register from the sensor
	 * Takes the address of the register as a 16 bit word
	 * Returns the value of the register
	 */
	uint16_t read(uint16_t registerAddress);
	
	/*
	 * Write to a register
	 * Takes the 16-bit  address of the target register and the 16 bit word of data
	 * to be written to that register
	 * Returns the value of the register after the write has been performed. This
	 * is read back from the sensor to ensure a sucessful write.
	 */
	uint16_t write(uint16_t registerAddress, uint16_t data);

	/**
	 * Returns the raw angle directly from the sensor
	 */
	uint16_t getRawRotation();

	/**
	 * Get the rotation of the sensor relative to the zero position.
	 *
	 * @return {int} between -2^13 and 2^13
	 */
	int getRotation();

	/**
	 * returns the value of the state register
	 * @return 16 bit word containing flags
	 */
	uint16_t getState();

	/*
	 * Check if an error has been encountered.
	 */
	uint8_t error();


	/**
	 * Returns the value used for Automatic Gain Control (Part of diagnostic
	 * register)
	 */
	uint8_t getGain();


	/*
	 * Get and clear the error register by reading it
	 */
	uint16_t getErrors();

	/*
	 * Set the zero position
	 */
	void setZeroPosition(uint16_t arg_position);

	/*
	 * Returns the current zero position
	 */
	uint16_t getZeroPosition();

	/*
	 * Returns normalized angle value
	 */
	float normalize(float angle);

	/*
	 * Returns calculated angle value
	 */
	float read2angle(uint16_t angle);

	/*
	 * Returns calculated angle value in radians
	 */
	float getAngleInRad();

	/*
	 * get current angular velocity (rad/s)
	 */
    float getVelocity();

    /*
     * set current agle as zero angle
     * return the angle [rad] difference
     */
    float initRelativeZero();
    /*
     * return the angle [rad] difference
     */
    float initAbsoluteZero();
    /*
     * returns 1 because it is the absolute sensor
     */
    int hasAbsoluteZero();
    /*
     * returns 0  maning it doesn't need search for absolute zero
     */
    int needsAbsoluteZeroSearch();

	private:

    // velocity calculation variables
    float angle_prev;
    uint32_t velocity_calc_timestamp;
    // total angle tracking variables
    float full_rotation_offset;
    float angle_data_prev;
    // zero offset
    uint16_t zero_offset;

	uint8_t spiCalcEvenParity(uint16_t value);
	
};


/* Private defines -----------------------------------------------------------*/

const int AS5048A_CLEAR_ERROR_FLAG              = 0x0001;
const int AS5048A_PROGRAMMING_CONTROL           = 0x0003;
const int AS5048A_OTP_REGISTER_ZERO_POS_HIGH    = 0x0016;
const int AS5048A_OTP_REGISTER_ZERO_POS_LOW     = 0x0017;
const int AS5048A_DIAG_AGC                      = 0x3FFD;
const int AS5048A_MAGNITUDE                     = 0x3FFE;
const int AS5048A_ANGLE                         = 0x3FFF;

#endif


#ifdef __cplusplus
}
#endif


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/