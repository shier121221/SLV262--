#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void MotorAll_Init(void);
void MotorR_SetSpeed(int16_t Speed);  // Speed范围: -1000 ~ +1000
void MotorL_SetSpeed(int16_t Speed);  // Speed范围: -1000 ~ +1000

#endif
