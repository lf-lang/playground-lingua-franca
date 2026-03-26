/*****************************************************************************
* | File        :   DEV_Config.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master 
*                and enhance portability
*----------------
* |	This version:   V1.0
* | Date        :   2018-09-03
* | Info        :   Basic version
*
******************************************************************************/
#ifndef __DEV_CONFIG_H_
#define __DEV_CONFIG_H_

#include <bcm2835.h>
#include <stdint.h>

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
 * GPIO
**/
#define M1_ENABLE_PIN 12
#define M1_DIR_PIN 13
#define M1_STEP_PIN 19
#define M1_M0_PIN 16
#define M1_M1_PIN 17
#define M1_M2_PIN 20

#define M2_ENABLE_PIN 4
#define M2_DIR_PIN 24
#define M2_STEP_PIN 18
#define M2_M0_PIN 21
#define M2_M1_PIN 22
#define M2_M2_PIN 27

#define DEV_Digital_Write(_pin, _value) bcm2835_gpio_write(_pin, _value)

/*------------------------------------------------------------------------------------------------------*/

UBYTE DEV_ModuleInit(void);
void DEV_ModuleExit(void);

void DEV_Delay_ms(UDOUBLE xms);
void DEV_Delay_us(UDOUBLE xus);

#endif