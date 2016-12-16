/**
 * @file         main.h
 * @version      1.0
 * @date         2015
 * @author       Christoph Lauer
 * @compiler     armcc
 * @copyright    Christoph Lauer engineering
 */

#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_audio_codec.h"
#include <stdio.h>
#include "stm32f4xx_it.h"
#include "headphone.h"
#include "microphone.h"
#include "pdm_filter.h"
#include "stm32f4xx_usart.h"

#include <stdint.h>
typedef struct {
	uint8_t x, y, z;
} Accelerometer;

// Accelerometer
void Accelerometer_GetData(Accelerometer * ac);
void SPI1_Init(void);
void SPI1_SendData(uint8_t adress, uint8_t data);
uint8_t SPI1_GetData(uint8_t adress);

#endif
