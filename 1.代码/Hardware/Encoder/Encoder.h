#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_TIM3_Init(void);      // TIM3硬件编码器初始化(PA6/PA7)
int16_t Encoder_TIM3_Get(void);    // 获取TIM3编码器增量值

#endif
