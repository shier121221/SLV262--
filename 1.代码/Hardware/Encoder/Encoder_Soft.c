/******************************TIM4硬件编码器模块**************************
使用TIM4硬件编码器模式(PB6/PB7引脚)
编码器A相 - PB6 (TIM4_CH1)
编码器B相 - PB7 (TIM4_CH2)
注意：PB0/PB1保持未配置状态(悬空)
*********************************************************************/
#include "stm32f10x.h"

/**
  * 函    数：TIM4硬件编码器初始化(PB6-CH1, PB7-CH2)
  * 参    数：无
  * 返 回 值：无
  */
void Encoder_Soft_Init(void)
{
    /* 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);   // 开启TIM4时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  // 开启GPIOB时钟
    
    /* GPIO初始化 - PB6和PB7配置为上拉输入 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;          // 上拉输入模式
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // PB6/PB7
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* 时基单元初始化 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               // ARR最大值
    TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;                // 不预分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
    
    /* 输入捕获初始化 */
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);                // 结构体默认值
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;       // 通道1(PB6)
    TIM_ICInitStructure.TIM_ICFilter = 0xF;                // 滤波器
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;       // 通道2(PB7)
    TIM_ICInitStructure.TIM_ICFilter = 0xF;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    /* 编码器接口配置 - 双边沿计数 */
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, 
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    /* 使能TIM4 */
    TIM_Cmd(TIM4, ENABLE);
}

/**
  * 函    数：获取TIM4编码器的增量值
  * 参    数：无
  * 返 回 值：自上次调用后的编码器增量值
  */
int16_t Encoder_Soft_Get(void)
{
    int16_t Temp;
    Temp = TIM_GetCounter(TIM4);  // 读取计数值
    TIM_SetCounter(TIM4, 0);      // 清零
    return Temp;
}
