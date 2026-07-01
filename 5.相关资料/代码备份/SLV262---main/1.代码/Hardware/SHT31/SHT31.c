#include "stm32f10x.h"
#include "Delay.h"
#include "MyI2C.h"
#include "SHT31.h"

#define SHT31_ADDRESS      0x88
#define SHT31_CMD_MEASURE  0x2C06

static uint8_t SHT31_CheckCrc(uint8_t *data, uint8_t crc)
{
    uint8_t i;
    uint8_t bit;
    uint8_t value = 0xFF;

    for (i = 0; i < 2; i++)
    {
        value ^= data[i];
        for (bit = 0; bit < 8; bit++)
        {
            if (value & 0x80)
            {
                value = (value << 1) ^ 0x31;
            }
            else
            {
                value <<= 1;
            }
        }
    }

    return (value == crc) ? MYI2C_OK : MYI2C_ERROR;
}

uint8_t SHT31_Init(void)
{
    return MYI2C_OK;
}

uint8_t SHT31_ReadData(float *temperature, float *humidity)
{
    uint8_t data[6];
    uint16_t rawTemp;
    uint16_t rawHumi;

    if (temperature == 0 || humidity == 0) return MYI2C_ERROR;

    if (MyI2C_WriteCommand(SHT31_ADDRESS, SHT31_CMD_MEASURE) != MYI2C_OK)
    {
        return MYI2C_ERROR;
    }

    Delay_ms(20);

    if (MyI2C_ReadBytes(SHT31_ADDRESS, data, 6) != MYI2C_OK)
    {
        return MYI2C_ERROR;
    }

    if (SHT31_CheckCrc(&data[0], data[2]) != MYI2C_OK) return MYI2C_ERROR;
    if (SHT31_CheckCrc(&data[3], data[5]) != MYI2C_OK) return MYI2C_ERROR;

    rawTemp = ((uint16_t)data[0] << 8) | data[1];
    rawHumi = ((uint16_t)data[3] << 8) | data[4];

    *temperature = -45.0f + 175.0f * (float)rawTemp / 65535.0f;
    *humidity = 100.0f * (float)rawHumi / 65535.0f;

    return MYI2C_OK;
}
