#ifndef __ENCODER_SOFT_H
#define __ENCODER_SOFT_H

void Encoder_Soft_Init(void);      // TIM4硬件编码器初始化(PB6/PB7)
int16_t Encoder_Soft_Get(void);    // 获取TIM4编码器增量值

#endif
