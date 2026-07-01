#ifndef __MYIIC_H
#define __MYIIC_H

#include "stm32f10x.h"
#include "delay.h"

//------------------- I2C 软件模拟引脚定义 -------------------
#define I2Cx_SCL_PIN            GPIO_Pin_8
#define I2Cx_SCL_GPIO_PORT      GPIOB
#define I2Cx_SCL_GPIO_CLK       RCC_APB2Periph_GPIOB

#define I2Cx_SDA_PIN            GPIO_Pin_9
#define I2Cx_SDA_GPIO_PORT      GPIOB
#define I2Cx_SDA_GPIO_CLK       RCC_APB2Periph_GPIOB

//------------------- I2C 引脚操作宏定义 -------------------
#define I2C_SCL_HIGH()      GPIO_SetBits(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN)
#define I2C_SCL_LOW()       GPIO_ResetBits(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN)

#define I2C_SDA_HIGH()      GPIO_SetBits(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN)
#define I2C_SDA_LOW()       GPIO_ResetBits(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN)

#define I2C_SDA_READ()      GPIO_ReadInputDataBit(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN)

void i2c_delay(void);				//I2C延时函数
void i2c_config(void);                //初始化IIC的IO口				 
void i2c_start(void);				//发送IIC开始信号
void i2c_stop(void);	  			//发送IIC停止信号
unsigned char i2c_send_byte(uint8_t data);			//IIC发送一个字节
uint8_t i2c_receive_byte(unsigned char ack);//IIC读取一个字节
u8 i2c_wait_ack(void); 				//IIC等待ACK信号
void i2c_ack(void);					//IIC发送ACK信号
void i2c_no_ack(void);				//IIC不发送ACK信号
uint8_t I2CReceiveAck(uint8_t timeout);//等待IIC接收ACK
void I2CReadBytes(int nBytes, char *dataP);
void i2c_Wait(int t);
#endif
















