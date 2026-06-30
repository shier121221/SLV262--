#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
#include "MPU6050.h"
#include "SHT31.h"
#include "MyI2C.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define SENSOR_TEST_PRINT_MS 500
#define GYRO_SENSITIVITY     16.4f
#define GYRO_Z_DEADZONE      0.5f

static float YawAngle = 0.0f;
static float GyroZ_Offset = 0.0f;

static void Debug_Printf(const char *format, ...)
{
    char buffer[160];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }
    if (len > (int)sizeof(buffer))
    {
        len = sizeof(buffer);
    }

    Serial1_SendArray((uint8_t *)buffer, (uint16_t)len);
}

static int16_t Float_To_Centi(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value * 100.0f + 0.5f);
    }
    return (int16_t)(value * 100.0f - 0.5f);
}

static void GyroZ_Calibrate(void)
{
    int32_t sum = 0;
    int16_t ax, ay, az, gx, gy, gz;
    uint16_t i;

    for (i = 0; i < 200; i++)
    {
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
        sum += gz;
        Delay_ms(5);
    }

    GyroZ_Offset = (float)sum / 200.0f;
}

static void Yaw_Update(int16_t gz, float dt)
{
    float gyro_z_dps = ((float)gz - GyroZ_Offset) / GYRO_SENSITIVITY;

    if (gyro_z_dps > -GYRO_Z_DEADZONE && gyro_z_dps < GYRO_Z_DEADZONE)
    {
        gyro_z_dps = 0.0f;
    }

    YawAngle += gyro_z_dps * dt;
    if (YawAngle > 180.0f) YawAngle -= 360.0f;
    if (YawAngle < -180.0f) YawAngle += 360.0f;
}

int main(void)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    float temp = 0.0f;
    float humi = 0.0f;
    uint8_t mpu_id;
    uint8_t sht_ok;
    uint8_t sht_err;
    uint16_t tick = 0;

    Serial1_Init();
    Delay_ms(200);

    Debug_Printf("\r\n==== SHT31 + MPU6050 sensor test ====\r\n");

    MPU6050_Init();
    Delay_ms(100);
    mpu_id = MPU6050_GetID();
    Debug_Printf("MPU6050 ID:0x%02X\r\n", mpu_id);

    SHT31_Init();
    Delay_ms(100);

    Debug_Printf("Keep car still, calibrating gyro...\r\n");
    GyroZ_Calibrate();
    Debug_Printf("GyroZ offset:%d.%02d\r\n",
                 Float_To_Centi(GyroZ_Offset) / 100,
                 Float_To_Centi(GyroZ_Offset) >= 0 ? Float_To_Centi(GyroZ_Offset) % 100 : -(Float_To_Centi(GyroZ_Offset) % 100));

    while (1)
    {
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
        Yaw_Update(gz, 0.01f);

        tick += 10;
        if (tick >= SENSOR_TEST_PRINT_MS)
        {
            tick = 0;
            sht_ok = (SHT31_ReadData(&temp, &humi) == 0);
            sht_err = SHT31_GetLastError();

            Debug_Printf(
                "AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d Yaw:%d.%02d SHT:%s Err:%u T:%d.%02d H:%d.%02d\r\n",
                ax, ay, az, gx, gy, gz,
                Float_To_Centi(YawAngle) / 100,
                Float_To_Centi(YawAngle) >= 0 ? Float_To_Centi(YawAngle) % 100 : -(Float_To_Centi(YawAngle) % 100),
                sht_ok ? "OK" : "FAIL",
                sht_err,
                Float_To_Centi(temp) / 100,
                Float_To_Centi(temp) >= 0 ? Float_To_Centi(temp) % 100 : -(Float_To_Centi(temp) % 100),
                Float_To_Centi(humi) / 100,
                Float_To_Centi(humi) >= 0 ? Float_To_Centi(humi) % 100 : -(Float_To_Centi(humi) % 100)
            );
        }

        Delay_ms(10);
    }
}
