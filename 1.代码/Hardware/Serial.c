#include "stm32f10x.h"
#include "Serial.h"
#include "Protocol.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define SERIAL1_BAUDRATE 921600
#define SERIAL1_PRINTF_ENABLE 0
#define SERIAL1_TX_DMA_SIZE 256
#ifndef SERIAL1_RX_DMA_SIZE
#define SERIAL1_RX_DMA_SIZE 256
#endif
#define SERIAL1_RX_FIFO_SIZE 256

char Serial1_RxPacket[100];
uint8_t Serial1_RxFlag = 0;
char Track_RxPacket[120];
uint8_t Track_RxFlag = 0;

static uint8_t Serial1_TxDmaBuffer[SERIAL1_TX_DMA_SIZE];
static volatile uint8_t Serial1_TxBusy = 0;
static uint8_t Serial1_RxDmaBuffer[SERIAL1_RX_DMA_SIZE];
static volatile uint16_t Serial1_RxDmaLastPos = 0;
static uint8_t Serial1_RxFifo[SERIAL1_RX_FIFO_SIZE];
static volatile uint16_t Serial1_RxFifoWrite = 0;
static volatile uint16_t Serial1_RxFifoRead = 0;
static volatile uint16_t Serial1_RxOverflow = 0;

static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;

    while (Y--)
    {
        Result *= X;
    }

    return Result;
}

static void Serial1_RxFifoPush(uint8_t data)
{
    uint16_t next = (uint16_t)((Serial1_RxFifoWrite + 1) % SERIAL1_RX_FIFO_SIZE);

    if (next == Serial1_RxFifoRead)
    {
        Serial1_RxOverflow++;
        return;
    }

    Serial1_RxFifo[Serial1_RxFifoWrite] = data;
    Serial1_RxFifoWrite = next;
}

static void Serial1_DMA_RxCollect(void)
{
    uint16_t pos = (uint16_t)(SERIAL1_RX_DMA_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5));

    while (Serial1_RxDmaLastPos != pos)
    {
        Serial1_RxFifoPush(Serial1_RxDmaBuffer[Serial1_RxDmaLastPos]);
        Serial1_RxDmaLastPos++;
        if (Serial1_RxDmaLastPos >= SERIAL1_RX_DMA_SIZE)
        {
            Serial1_RxDmaLastPos = 0;
        }
    }
}

void Serial1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = SERIAL1_BAUDRATE;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

    DMA_DeInit(DMA1_Channel4);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial1_TxDmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);

    DMA_DeInit(DMA1_Channel5);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial1_RxDmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = SERIAL1_RX_DMA_SIZE;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel5, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
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

uint8_t Serial1_DMA_SendArray(const uint8_t *Array, uint16_t Length)
{
    uint32_t wait = 0;

    if (Array == 0 || Length == 0 || Length > SERIAL1_TX_DMA_SIZE)
    {
        return 1;
    }

    while (Serial1_TxBusy && wait < 100000)
    {
        wait++;
    }

    if (Serial1_TxBusy)
    {
        return 2;
    }

    memcpy(Serial1_TxDmaBuffer, Array, Length);
    Serial1_TxBusy = 1;

    DMA_Cmd(DMA1_Channel4, DISABLE);
    DMA_SetCurrDataCounter(DMA1_Channel4, Length);
    DMA_ClearFlag(DMA1_FLAG_GL4);
    DMA_Cmd(DMA1_Channel4, ENABLE);

    return 0;
}

uint16_t Serial1_DMA_Read(uint8_t *Buffer, uint16_t MaxLength)
{
    uint16_t count = 0;

    if (Buffer == 0 || MaxLength == 0)
    {
        return 0;
    }

    Serial1_DMA_RxCollect();

    while (Serial1_RxFifoRead != Serial1_RxFifoWrite && count < MaxLength)
    {
        Buffer[count++] = Serial1_RxFifo[Serial1_RxFifoRead];
        Serial1_RxFifoRead = (uint16_t)((Serial1_RxFifoRead + 1) % SERIAL1_RX_FIFO_SIZE);
    }

    return count;
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
#if SERIAL1_PRINTF_ENABLE
    char String[100];
    va_list arg;

    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);

    Serial1_SendString(String);
#else
    (void)format;
#endif
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
    if (USART_GetITStatus(USART1, USART_IT_IDLE) == SET)
    {
        (void)USART1->SR;
        (void)USART1->DR;
        Serial1_DMA_RxCollect();
    }
}

void DMA1_Channel4_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC4);
        DMA_Cmd(DMA1_Channel4, DISABLE);
        Serial1_TxBusy = 0;
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

static uint8_t Protocol_TxFrame[PROTOCOL_MAX_FRAME];

uint8_t Protocol_SendFrame(uint8_t msgId, const uint8_t *payload, uint8_t len)
{
    uint8_t checksum = msgId + len;
    uint8_t i;

    if (len > PROTOCOL_MAX_PAYLOAD)
    {
        return 1;
    }
    if (len > 0 && payload == 0)
    {
        return 2;
    }

    Protocol_TxFrame[0] = PROTOCOL_HEAD;
    Protocol_TxFrame[1] = msgId;
    Protocol_TxFrame[2] = len;

    for (i = 0; i < len; i++)
    {
        Protocol_TxFrame[3 + i] = payload[i];
        checksum += payload[i];
    }
    Protocol_TxFrame[3 + len] = checksum;

    return Serial1_DMA_SendArray(Protocol_TxFrame, (uint16_t)(len + 4));
}

__weak void Protocol_OnFrame(uint8_t msgId, const uint8_t *payload, uint8_t len)
{
    (void)msgId;
    (void)payload;
    (void)len;
}

void Protocol_Poll(void)
{
    static uint8_t state = 0;
    static uint8_t msgId = 0;
    static uint8_t len = 0;
    static uint8_t index = 0;
    static uint8_t checksum = 0;
    static uint8_t payload[PROTOCOL_MAX_PAYLOAD];
    uint8_t buffer[32];
    uint16_t count;
    uint16_t i;

    count = Serial1_DMA_Read(buffer, sizeof(buffer));
    for (i = 0; i < count; i++)
    {
        uint8_t data = buffer[i];

        switch (state)
        {
            case 0:
                if (data == PROTOCOL_HEAD)
                {
                    state = 1;
                }
                break;

            case 1:
                msgId = data;
                checksum = data;
                state = 2;
                break;

            case 2:
                len = data;
                checksum += data;
                index = 0;
                if (len > PROTOCOL_MAX_PAYLOAD)
                {
                    state = 0;
                }
                else
                {
                    state = (len == 0) ? 4 : 3;
                }
                break;

            case 3:
                payload[index++] = data;
                checksum += data;
                if (index >= len)
                {
                    state = 4;
                }
                break;

            case 4:
                if (data == checksum)
                {
                    Protocol_OnFrame(msgId, payload, len);
                }
                state = 0;
                break;

            default:
                state = 0;
                break;
        }
    }
}
