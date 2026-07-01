#ifndef __MYI2C_H
#define __MYI2C_H

#include "stm32f10x.h"

#define MYI2C_OK       0
#define MYI2C_ERROR    1

void MyI2C_Init(void);
uint8_t MyI2C_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data);
uint8_t MyI2C_ReadReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t *Data);
uint8_t MyI2C_ReadRegs(uint8_t DevAddress, uint8_t RegAddress, uint8_t *Data, uint16_t Length);
uint8_t MyI2C_WriteCommand(uint8_t DevAddress, uint16_t Command);
uint8_t MyI2C_ReadBytes(uint8_t DevAddress, uint8_t *Data, uint16_t Length);

#endif
