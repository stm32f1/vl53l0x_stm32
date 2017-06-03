#ifndef DEVICE_H
#define DEVICE_H

#pragma GCC diagnostic ignored "-Wsign-conversion"
#include <misc.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_dac.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_tim.h>
#pragma GCC diagnostic warning "-Wsign-conversion"

//#define USE_SLEEP

#include "../config.h"
#include "conversion.h"
#include "../port.h"

#define BITBAND_GPIO(bitband_addr, pin)	(*(volatile uint32_t *)(PERIPH_BB_BASE + ((intptr_t)bitband_addr - PERIPH_BASE)*32 + 4 * pin))

#define SYS_FREQ	72000000
#define Periph_enable_flag  0x01
#define CAR_HALF_WIDTH	45	//mm
#define REAR_OVERHANG 55 //mm
#define I2C_CLOCK   50000

//typedef enum{FALSE, TRUE}FLAG;

#endif /* DEVICE_H */
