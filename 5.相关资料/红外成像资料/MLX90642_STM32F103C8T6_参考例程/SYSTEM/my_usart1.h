#ifndef __MYUSART_H  // 条件编译，防止重复引用
#define __MYUSART_H
#include "stm32f10x.h"
#include "stdio.h"
#include "mlx90614.h"
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收


#define RCC_USAR_RxTx   RCC_APB1Periph_USART3
#define RCC_GPIOx       RCC_APB2Periph_GPIOB
#define GPIO_Portx			GPIOB
#define GPIO_Pin_Tx		  GPIO_Pin_10
#define GPIO_Pin_Rx		  GPIO_Pin_11
#define USART_x         USART3
#define USARTx_brr			115200
#define USARTx_IR    	  USART3_IRQn
#define USARTx_IRQHand  USART3_IRQHandler
typedef struct
{
	uint8_t buf[30];
	uint8_t flag;
	uint32_t count;
}_USARTx_GetData;
 
typedef struct
{
	unsigned char set_state;
	//直流无刷电机设置
	float BLDC_duty;
	uint8_t BLDC_fre;
	uint8_t BLDC_v;
	unsigned char BLDC_ready;
	//步进电机设置
	float STEP_duty;
	uint8_t STEP_fre;
	uint8_t STEP_count;
	unsigned char STEP_ready;
	unsigned char STEP_direction;
	uint8_t STEP_count_mode;
}_uart_read_set;
extern _uart_read_set uart_read_set;
extern _USARTx_GetData USARTx_GetData;
void USART3_Init(void);
void USART_SendByte(uint8_t Byte);
void USART_SendString(char * str);
void USART_SendBuff(char *str,uint32_t len);
uint8_t USART_GetByte(void);
//void RCC_Configuration(void);
void USART_Config(void);
void USART_NVIC_Config(void) ; 
void process_command(uint8_t *data, uint16_t len);
void shujuti_tx(void);
void start_tx(float temperature);
#endif
