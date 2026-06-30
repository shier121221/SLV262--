#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include <stdio.h>

extern char Serial1_RxPacket[100];
extern uint8_t Serial1_RxFlag;
extern char Track_RxPacket[120];
extern uint8_t Track_RxFlag;

#define SERIAL1_RX_DMA_SIZE 256

void Serial1_Init(void);
void Serial1_SendByte(uint8_t Byte);
void Serial1_SendString(char *String);
void Serial1_SendArray(uint8_t *Array, uint16_t Length);
uint8_t Serial1_DMA_SendArray(const uint8_t *Array, uint16_t Length);
uint16_t Serial1_DMA_Read(uint8_t *Buffer, uint16_t MaxLength);
void Serial1_SendHex(uint8_t Byte);
void Serial1_SendHexArray(uint8_t *Array, uint16_t Length, uint8_t Space);
void Serial1_SendNumber(uint32_t Number, uint8_t Length);
void Serial1_Printf(char *format, ...);
void Track_USART2_Init(void);
void Track_SendByte(uint8_t Byte);
void Track_SendString(char *String);
void Track_RequestDigitalAnalog(void);

#endif
