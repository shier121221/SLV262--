#ifndef __SHT31_H
#define __SHT31_H

#include "stm32f10x.h"

uint8_t SHT31_Init(void);
uint8_t SHT31_ReadData(float *temperature, float *humidity);

#endif
