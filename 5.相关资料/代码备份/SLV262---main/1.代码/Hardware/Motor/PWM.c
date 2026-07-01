#include "stm32f10x.h"   // Device header

//初始化定时器TIM2 并配置PWM初始化参数
//PA0-TIM2_CH1(PWMB), PA1-TIM2_CH2(PWMA)
void PWM12_Init(void)
{    
    //开启TIM2和GPIOA的时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    //初始化GPIO口 PA0 PA1 用于产生PWM信号
    GPIO_InitTypeDef GPIO_InitStructure;
    //复用推挽输出模式因为复用了TIM2外设
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO A0 A1 
    //使用内部时钟
    TIM_InternalClockConfig(TIM2);
    //配置时基单元参数
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;//定时器不分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;//ARR自动重装值(0-999对应0-1000)
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;//PSC预分频值(72MHz/72/1000=1kHz)
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//到达ARR触发一次中断 停止计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);//初始化单元
    //输出比较结构体配置
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);//补全结构体中未配置参数
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//选择PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//输出比较极性选择
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;//输出使能

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);//初始化 TIM2 OC1 (PA0-PWMB)
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);//使能CCR1自动重装
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);//初始化 TIM2 OC2 (PA1-PWMA)
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);//使能CCR2自动重装
    
    TIM_ARRPreloadConfig(TIM2, ENABLE);//开启预装载
    TIM_Cmd(TIM2, ENABLE);//开启定时器2
    TIM2->CCR1 = 0;//设置输出比较值
    TIM2->CCR2 = 0;
}

//设置PWMB比较值(PA0-TIM2_CH1)
void PWM12_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM2, Compare);
}

//设置PWMA比较值(PA1-TIM2_CH2)
void PWM12_SetCompare2(uint16_t Compare)
{
    TIM_SetCompare2(TIM2, Compare);
}
