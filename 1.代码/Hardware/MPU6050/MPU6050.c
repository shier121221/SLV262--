#include "stm32f10x.h"
#include "MyI2C.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS     0xD0

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MyI2C_WriteReg(MPU6050_ADDRESS, RegAddress, Data);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data = 0;

    MyI2C_ReadReg(MPU6050_ADDRESS, RegAddress, &Data);
    return Data;
}

void MPU6050_Init(void)
{
    MyI2C_Init();

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t data[14];

    if (MyI2C_ReadRegs(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, data, 14) != MYI2C_OK)
    {
        *AccX = 0;
        *AccY = 0;
        *AccZ = 0;
        *GyroX = 0;
        *GyroY = 0;
        *GyroZ = 0;
        return;
    }

    *AccX = (int16_t)((data[0] << 8) | data[1]);
    *AccY = (int16_t)((data[2] << 8) | data[3]);
    *AccZ = (int16_t)((data[4] << 8) | data[5]);
    *GyroX = (int16_t)((data[8] << 8) | data[9]);
    *GyroY = (int16_t)((data[10] << 8) | data[11]);
    *GyroZ = (int16_t)((data[12] << 8) | data[13]);
}
