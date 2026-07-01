/******************************Motor电机模块**************************
电机初始化设置 以及电机PWM设置
TB6612引脚配置:
  PWMB - PA0 (TIM2_CH1)
  PWMA - PA1 (TIM2_CH2)
  AIN1 - PB14
  AIN2 - PB15
  BIN1 - PB13
  BIN2 - PB12
*********************************************************************/
#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void MotorAll_Init(void)
{
    //开启电机驱动口的GPIO时钟 (PB12-PB15)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    PWM12_Init();//开启定时器PWM
}

//设置左路电机(A电机)速度 PWM - AIN1(PB14), AIN2(PB15), PWMA(PA1)
//Speed范围: -1000 ~ +1000
void MotorL_SetSpeed(int16_t Speed)
{
    if (Speed >= 0)//Speed值为正
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);   //AIN1=1
        GPIO_ResetBits(GPIOB, GPIO_Pin_15); //AIN2=0 电机正转
        PWM12_SetCompare2(Speed);//设置Speed转速
    }
    else//Speed值为负
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14); //AIN1=0
        GPIO_SetBits(GPIOB, GPIO_Pin_15);   //AIN2=1 电机反转
        PWM12_SetCompare2(-Speed);//设为-Speed转速
    }
}

//设置右路电机(B电机)速度 PWM - BIN1(PB13), BIN2(PB12), PWMB(PA0)
//Speed范围: -1000 ~ +1000
void MotorR_SetSpeed(int16_t Speed)
{
    if (Speed >= 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_13);   //BIN1=1
        GPIO_ResetBits(GPIOB, GPIO_Pin_12); //BIN2=0 电机正转
        PWM12_SetCompare1(Speed);
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_13); //BIN1=0
        GPIO_SetBits(GPIOB, GPIO_Pin_12);   //BIN2=1 电机反转
        PWM12_SetCompare1(-Speed);
    }
}

