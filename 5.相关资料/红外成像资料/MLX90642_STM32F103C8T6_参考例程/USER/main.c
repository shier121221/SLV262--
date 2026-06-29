/**
 * @file    main.c
 * @brief   驱动 MLX90642 32*24 实例例程
 * @author 	Litus_zxg
 * @date    2025-07-03
 * @version v1.0
 *
 * @details
 * - 本文件用于初始化 MLX90642 探头（通过 I2C 配置寄存器）
 * - 提供探头采集温度的基本控制接口
 */

#include "stm32f10x.h"
#include "delay.h"
#include "usart.h"
#include "MLX90642.h"
#include "MLX90642_depends.h"
#include "myiic.h"

#define MLX90642_ADDR_DEFAULT         SA_90642_DEFAULT
#define MLX90642_ADDR_FALLBACK        0x33
#define MLX90642_WAIT_READY_RETRY_MAX 200

uint16_t rawData[769]; // 768像素+1校验
float tempMap[768];

static uint8_t MLX90642_DetectAddress(void)
{
    uint16_t id[MLX90642_NUMBER_OF_ID_WORDS] = {0};

    if(MLX90642_GetID(MLX90642_ADDR_DEFAULT, id) == 0) {
        printf("MLX90642 addr probe ok: 0x%02X\r\n", MLX90642_ADDR_DEFAULT);
        return MLX90642_ADDR_DEFAULT;
    }

    if(MLX90642_GetID(MLX90642_ADDR_FALLBACK, id) == 0) {
        printf("MLX90642 addr probe ok: 0x%02X\r\n", MLX90642_ADDR_FALLBACK);
        return MLX90642_ADDR_FALLBACK;
    }

    printf("MLX90642 addr probe failed: tried 0x%02X and 0x%02X\r\n",
           MLX90642_ADDR_DEFAULT,
           MLX90642_ADDR_FALLBACK);
    return 0;
}

int main(void)
{
    uint16_t i = 0;
    uint16_t row = 0;
    uint16_t col = 0;
    uint16_t retry = 0;
    uint8_t mlxAddr = 0;
    int status = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
    uart_init(115200);
    i2c_config();

    printf("MLX90642 boot...\r\n");
    printf("I2C idle level: SCL=%d SDA=%d\r\n", I2C_SCL_READ(), I2C_SDA_READ());

    mlxAddr = MLX90642_DetectAddress();
    if(mlxAddr == 0) {
        while(1);
    }

    MLX90642_WakeUp(mlxAddr);
    status = MLX90642_Init(mlxAddr);
    if(status < 0)
    {
        printf("MLX90642_Init failed: %d\r\n", status);
        while(1);
    }

    status = MLX90642_SetMeasMode(mlxAddr, MLX90642_CONT_MEAS_MODE);
    if(status < 0)
    {
        printf("SetMeasMode failed: %d\r\n", status);
        while(1);
    }

    status = MLX90642_SetRefreshRate(mlxAddr, MLX90642_REF_RATE_8HZ);
    if(status < 0)
    {
        printf("SetRefreshRate failed: %d\r\n", status);
        while(1);
    }

    printf("Litus_MLX90642 init ok\r\n");

    while (1)
    {
        retry = 0;
        do {
            status = MLX90642_IsReadWindowOpen(mlxAddr);
            if(status < 0) {
                printf("ReadWindow error: %d\r\n", status);
                delay_ms(1000);
                break;
            }

            delay_ms(5);
            retry++;
        } while((status == MLX90642_NO) && (retry < MLX90642_WAIT_READY_RETRY_MAX));

        if(status < 0) {
            continue;
        }

        if(retry >= MLX90642_WAIT_READY_RETRY_MAX) {
            printf("Wait data ready timeout\r\n");
            delay_ms(200);
            continue;
        }

        status = MLX90642_GetImage(mlxAddr, rawData);
        if(status < 0) {
            printf("Read Error: %d\r\n", status);
            continue;
        }

        for(i = 0; i < 768; i++) {
            tempMap[i] = rawData[i] * 0.02f;
        }

        printf("----- Temperature Matrix -----\r\n");
        for(row = 0; row < 24; row++) {
            for(col = 0; col < 32; col++) {
                printf("%5.2f  ", tempMap[row * 32 + col]);
            }
            printf("\r\n");
        }
    }
}
