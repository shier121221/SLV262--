#include "MLX90642_App.h"
#include "MLX90642.h"
#include "MLX90642_depends.h"
#include "Serial.h"
#include "myiic.h"

#define MLX90642_APP_ADDR_DEFAULT   SA_90642_DEFAULT
#define MLX90642_APP_ADDR_FALLBACK  0x33

static uint16_t MLX90642_RawData[MLX90642_APP_PIXELS];
static uint8_t MLX90642_AppAddr = MLX90642_APP_ADDR_DEFAULT;

static uint8_t MLX90642_App_ProbeAddress(uint8_t slaveAddr)
{
    uint8_t ack;

    i2c_start();
    ack = i2c_send_byte(slaveAddr << 1);
    i2c_stop();

    return (ack == 0) ? 1 : 0;
}

static void MLX90642_App_ScanBus(void)
{
    uint8_t addr;
    uint8_t found = 0;

    for (addr = SA_90642_MIN; addr <= SA_90642_MAX; addr++)
    {
        if (MLX90642_App_ProbeAddress(addr))
        {
            found = 1;
            Serial1_Printf("MLX90642 bus addr found:0x%02X\r\n", addr);
        }
    }

    if (found == 0)
    {
        Serial1_Printf("MLX90642 bus scan:no ack\r\n");
    }
}

static int MLX90642_App_InitDevice(uint8_t slaveAddr)
{
    uint16_t refTime;
    int status;
    int pollTries;

    status = MLX90642_GetRefreshTime(slaveAddr);
    if (status < 0)
    {
        Serial1_Printf("MLX90642 refresh time failed:%d\r\n", status);
        return status;
    }

    refTime = (uint16_t)status;

    status = MLX90642_ClearDataReady(slaveAddr);
    if (status != 0)
    {
        Serial1_Printf("MLX90642 clear ready failed:%d\r\n", status);
        return -MLX90642_INVAL_VAL_ERR;
    }

    status = MLX90642_StartSync(slaveAddr);
    if (status < 0)
    {
        Serial1_Printf("MLX90642 start sync failed:%d\r\n", status);
        return status;
    }

    MLX90642_Wait_ms(refTime);

    for (pollTries = 0; pollTries < MLX90642_MAX_POLL_TRIES; pollTries++)
    {
        MLX90642_Wait_ms(MLX90642_POLL_TIME_MS);
        status = MLX90642_IsDataReady(slaveAddr);
        if (status < 0)
        {
            Serial1_Printf("MLX90642 data ready read failed:%d\r\n", status);
            return status;
        }

        if (status == MLX90642_YES)
        {
            return 0;
        }
    }

    Serial1_Printf("MLX90642 data ready timeout\r\n");
    return -MLX90642_TIMEOUT_ERR;
}

uint8_t MLX90642_App_Init(void)
{
    int status;
    uint16_t mlxid[MLX90642_NUMBER_OF_ID_WORDS];

    i2c_config();

    MLX90642_WakeUp(MLX90642_APP_ADDR_DEFAULT);
    status = MLX90642_GetID(MLX90642_APP_ADDR_DEFAULT, mlxid);
    if (status == 0)
    {
        MLX90642_AppAddr = MLX90642_APP_ADDR_DEFAULT;
    }
    else
    {
        MLX90642_WakeUp(MLX90642_APP_ADDR_FALLBACK);
        status = MLX90642_GetID(MLX90642_APP_ADDR_FALLBACK, mlxid);
        if (status == 0)
        {
            MLX90642_AppAddr = MLX90642_APP_ADDR_FALLBACK;
        }
    }

    if (status < 0)
    {
        Serial1_Printf("MLX90642 id read failed:%d\r\n", status);
        MLX90642_App_ScanBus();
        return 1;
    }

    Serial1_Printf("MLX90642 addr:0x%02X\r\n", MLX90642_AppAddr);
    Serial1_Printf("MLX90642 ID:%04X%04X%04X%04X\r\n",
                   mlxid[0], mlxid[1], mlxid[2], mlxid[3]);

    status = MLX90642_App_InitDevice(MLX90642_AppAddr);
    if (status < 0)
    {
        Serial1_Printf("MLX90642 init failed:%d\r\n", status);
        return 1;
    }

    status = MLX90642_SetOutputFormat(MLX90642_AppAddr, MLX90642_TEMPERATURE_OUTPUT);
    if (status < 0)
    {
        Serial1_Printf("MLX90642 output fmt failed:%d\r\n", status);
        return 1;
    }

    status = MLX90642_SetMeasMode(MLX90642_AppAddr, MLX90642_CONT_MEAS_MODE);
    if (status < 0)
    {
        Serial1_Printf("MLX90642 meas mode failed:%d\r\n", status);
        return 1;
    }

    status = MLX90642_SetRefreshRate(MLX90642_AppAddr, MLX90642_REF_RATE_8HZ);
    if (status < 0)
    {
        Serial1_Printf("MLX90642 refresh failed:%d\r\n", status);
        return 1;
    }

    Serial1_Printf("MLX90642 init ok\r\n");
    return 0;
}

uint8_t MLX90642_App_ReadFrame(float *temperatureMap)
{
    uint16_t i;
    int status;

    if (temperatureMap == 0)
    {
        return 1;
    }

    status = MLX90642_IsReadWindowOpen(MLX90642_AppAddr);
    if (status != MLX90642_YES)
    {
        return 2;
    }

    status = MLX90642_GetImage(MLX90642_AppAddr, MLX90642_RawData);
    if (status < 0)
    {
        Serial1_Printf("MLX90642 read failed:%d\r\n", status);
        return 3;
    }

    for (i = 0; i < MLX90642_APP_PIXELS; i++)
    {
        temperatureMap[i] = (float)MLX90642_RawData[i] * 0.02f;
    }

    return 0;
}

void MLX90642_App_PrintFrame(float *temperatureMap)
{
    uint8_t row;
    uint8_t col;

    if (temperatureMap == 0)
    {
        return;
    }

    Serial1_Printf("----- MLX90642 Temperature Matrix -----\r\n");
    for (row = 0; row < MLX90642_APP_HEIGHT; row++)
    {
        for (col = 0; col < MLX90642_APP_WIDTH; col++)
        {
            Serial1_Printf("%5.2f ", temperatureMap[row * MLX90642_APP_WIDTH + col]);
        }
        Serial1_Printf("\r\n");
    }
}
