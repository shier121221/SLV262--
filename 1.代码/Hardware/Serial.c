#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

char Serial1_RxPacket[100];
uint8_t Serial1_RxFlag = 0;
char Track_RxPacket[120];
uint8_t Track_RxFlag = 0;

static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;

    while (Y--)
    {
        Result *= X;
    }

    return Result;
}

void Serial1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);
}

void Serial1_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial1_SendString(char *String)
{
    uint16_t i;

    for (i = 0; String[i] != '\0'; i++)
    {
        Serial1_SendByte(String[i]);
    }
}

void Serial1_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;

    for (i = 0; i < Length; i++)
    {
        Serial1_SendByte(Array[i]);
    }
}

void Serial1_SendHex(uint8_t Byte)
{
    char HexChars[] = "0123456789ABCDEF";

    Serial1_SendByte(HexChars[(Byte >> 4) & 0x0F]);
    Serial1_SendByte(HexChars[Byte & 0x0F]);
}

void Serial1_SendHexArray(uint8_t *Array, uint16_t Length, uint8_t Space)
{
    uint16_t i;

    for (i = 0; i < Length; i++)
    {
        Serial1_SendHex(Array[i]);
        if (Space && i < Length - 1)
        {
            Serial1_SendByte(' ');
        }
    }
}

void Serial1_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;

    for (i = 0; i < Length; i++)
    {
        Serial1_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

void Serial1_Printf(char *format, ...)
{
    char String[100];
    va_list arg;

    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);

    Serial1_SendString(String);
}

void Track_USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

void Track_SendByte(uint8_t Byte)
{
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void Track_SendString(char *String)
{
    uint16_t i;

    for (i = 0; String[i] != '\0'; i++)
    {
        Track_SendByte(String[i]);
    }
}

void Track_RequestDigitalAnalog(void)
{
    Track_SendString("$0,1,1#");
}

void USART1_IRQHandler(void)
{
    static uint8_t pRxPacket = 0;
    static char tempBuffer[100];

    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART1);

        if (RxData == '\r')
        {
        }
        else if (RxData == '\n')
        {
            tempBuffer[pRxPacket] = '\0';

            if (pRxPacket > 0 && Serial1_RxFlag == 0)
            {
                strcpy(Serial1_RxPacket, tempBuffer);
                Serial1_RxFlag = 1;
            }

            pRxPacket = 0;
        }
        else
        {
            if (pRxPacket < 99)
            {
                tempBuffer[pRxPacket++] = RxData;
            }
            else
            {
                pRxPacket = 0;
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void USART2_IRQHandler(void)
{
    static uint8_t rxIndex = 0;
    static uint8_t rxStarted = 0;
    static char tempBuffer[120];

    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART2);

        if (RxData == '$')
        {
            rxStarted = 1;
            rxIndex = 0;
            tempBuffer[rxIndex++] = RxData;
        }
        else if (rxStarted)
        {
            if (rxIndex < 119)
            {
                tempBuffer[rxIndex++] = RxData;
            }
            else
            {
                rxStarted = 0;
                rxIndex = 0;
            }

            if (RxData == '#')
            {
                tempBuffer[rxIndex] = '\0';
                if (Track_RxFlag == 0)
                {
                    strcpy(Track_RxPacket, tempBuffer);
                    Track_RxFlag = 1;
                }
                rxStarted = 0;
                rxIndex = 0;
            }
        }

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

int fputc(int ch, FILE *f)
{
    Serial1_SendByte((uint8_t)ch);
    return ch;
}
