#ifndef __ESP01S_H
#define __ESP01S_H

#include "stm32f10x.h"

/*
 * ESP01S WiFi模块驱动 (Arduino编程版)
 * 说明：ESP01S已通过Arduino IDE编程，自动连接WiFi和服务器
 *       STM32只需通过串口2进行数据透传
 * 接线：ESP01S TX -> PA3, ESP01S RX -> PA2
 * 波特率：9600
 */

// 外部变量声明
extern char ESP_RxPacket[100];       // WiFi接收数据包(服务器发来的命令)
extern uint8_t ESP_RxFlag;           // WiFi接收标志位
extern uint8_t ESP_Connected;        // 服务器连接状态(由ESP01S上报)

// 函数声明
void ESP01S_Init(void);              // 初始化串口2
void ESP01S_SendByte(uint8_t Byte);  // 发送单字节
void ESP01S_SendString(char *String);// 发送字符串
void ESP01S_SendData(char *data);    // 发送数据到服务器
void ESP01S_Printf(char *format, ...);// 格式化发送

#endif
