#include "stm32f10x.h"                  // Device header

/**
  * 函    数：定时中断初始化(TIM1)
  * 参    数：无
  * 返 回 值：无
  * 说    明：使用TIM1产生定时中断，周期10ms(100Hz)，用于PID控制
  */
void Timer_Init(void)
{
	/*开启时钟 - TIM1在APB2总线上*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);			//开启TIM1的时钟
	
	/*配置时钟源*/
	TIM_InternalClockConfig(TIM1);		//选择TIM1为内部时钟
	
	/*时基单元初始化*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟分频，选择不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;				//计数周期，即ARR的值(1000次)
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720 - 1;				//预分频器，即PSC的值(72MHz/720=100kHz)
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;			//重复计数器，高级定时器需要设置
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);				//配置TIM1的时基单元(100kHz/1000=100Hz,即10ms)
	
	/*中断输出配置*/
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);						//清除定时器更新标志位
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);					//开启TIM1的更新中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//配置NVIC为分组2
	
	/*NVIC配置 - TIM1更新中断*/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;			//TIM1更新中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//响应优先级1
	NVIC_Init(&NVIC_InitStructure);
	
	/*TIM使能*/
	TIM_Cmd(TIM1, ENABLE);			//使能TIM1，定时器开始运行
}

/* 定时器中断函数，可以复制到使用它的地方
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}
*/
