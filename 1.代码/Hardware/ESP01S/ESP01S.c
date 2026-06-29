#include "stm32f10x.h"
#include "ESP01S.h"
#include "Delay.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

// 外部变量定义
char ESP_RxPacket[100];              // WiFi接收数据包
uint8_t ESP_RxFlag = 0;              // WiFi接收标志位
uint8_t ESP_Connected = 0;           // WiFi/服务器连接状态

/**
  * 函    数：ESP01S初始化(使用USART2, PA2-TX, PA3-RX)
  * 说    明：ESP01S已通过Arduino编程，STM32只需串口通信
  * 参    数：无
  * 返 回 值：无
  */
void ESP01S_Init(void)
{
    // 开启时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // GPIO初始化
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PA2 - USART2_TX 复用推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // PA3 - USART2_RX 上拉输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // USART2初始化 - 9600波特率(与ESP01S Arduino代码一致)
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);
    
    // 开启接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    
    // NVIC配置
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能USART2
    USART_Cmd(USART2, ENABLE);
    
    // 默认设置为未连接，等待ESP01S上报状态
    ESP_Connected = 0;
}

/**
  * 函    数：发送单字节
  * 参    数：Byte 要发送的字节
  * 返 回 值：无
  */
void ESP01S_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/**
  * 函    数：发送字符串
  * 参    数：String 要发送的字符串
  * 返 回 值：无
  */
void ESP01S_SendString(char *String)
{
    uint16_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        ESP01S_SendByte(String[i]);
    }
}

/**
  * 函    数：发送数据(透传模式，ESP01S已通过Arduino编程)
  * 参    数：data 要发送的数据
  * 返 回 值：无
  */
void ESP01S_SendData(char *data)
{
    if (ESP_Connected)
    {
        ESP01S_SendString(data);
    }
}

/**
  * 函    数：格式化发送数据
  * 参    数：format 格式化字符串
  * 参    数：... 可变参数
  * 返 回 值：无
  */
void ESP01S_Printf(char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    ESP01S_SendData(String);
}

/**
  * 函    数：USART2中断处理函数
  * 参    数：无
  * 返 回 值：无
  * 说    明：处理ESP01S的数据接收，解析状态消息和服务器命令
  */
void USART2_IRQHandler(void)
{
    static uint8_t pRxPacket = 0;
    static char tempBuffer[100];
    
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART2);
        
        // 状态机解析: 以换行符结尾的数据
        if (RxData == '\r')
        {
            // 忽略回车符
        }
        else if (RxData == '\n')
        {
            // 收到换行符，处理完整的一行数据
            tempBuffer[pRxPacket] = '\0';
            
            if (pRxPacket > 0)
            {
                // 检查是否是ESP01S状态消息
                if (strcmp(tempBuffer, "@SERVER_OK") == 0)
                {
                    ESP_Connected = 1;  // 服务器连接成功
                }
                else if (strcmp(tempBuffer, "@SERVER_FAIL") == 0 || 
                         strcmp(tempBuffer, "@WIFI_FAIL") == 0)
                {
                    ESP_Connected = 0;  // 连接失败
                }
                else if (strcmp(tempBuffer, "@WIFI_OK") == 0 ||
                         strcmp(tempBuffer, "@WIFI_CONNECTING") == 0 ||
                         strcmp(tempBuffer, "@SERVER_CONNECTING") == 0)
                {
                    // 状态消息，不处理
                }
                else if (tempBuffer[0] == '@' && tempBuffer[1] == 'I' && tempBuffer[2] == 'P')
                {
                    // IP地址消息，不处理
                }
                else if (ESP_RxFlag == 0)
                {
                    // 服务器发来的命令，复制到接收包
                    strcpy(ESP_RxPacket, tempBuffer);
                    ESP_RxFlag = 1;
                }
            }
            pRxPacket = 0;
        }
        else
        {
            // 普通字符，存入缓冲区
            if (pRxPacket < 99)
            {
                tempBuffer[pRxPacket++] = RxData;
            }
        }
        
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
