#ifndef __MLX90642_APP_H
#define __MLX90642_APP_H

#include "stm32f10x.h"

#define MLX90642_APP_WIDTH   32
#define MLX90642_APP_HEIGHT  24
#define MLX90642_APP_PIXELS  (MLX90642_APP_WIDTH * MLX90642_APP_HEIGHT)

uint8_t MLX90642_App_Init(void);
uint8_t MLX90642_App_ReadFrame(float *temperatureMap);
void MLX90642_App_PrintFrame(float *temperatureMap);

#endif
