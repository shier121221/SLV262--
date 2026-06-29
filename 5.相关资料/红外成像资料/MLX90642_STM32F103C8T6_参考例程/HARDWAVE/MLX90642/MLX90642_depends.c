#include "MLX90642_depends.h"
#include "stm32f10x.h"
#include "myiic.h"
#include "MLX90642.h"

int MLX90642_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                    uint16_t nMemAddressRead, uint16_t *rData)
{
    uint16_t i = 0;

    i2c_start();

    // Block read: START + SA(W) + address + RESTART + SA(R)
    if(i2c_send_byte(slaveAddr << 1)) {
        i2c_stop();
        return -1;
    }

    if(i2c_send_byte(startAddress >> 8) ||
       i2c_send_byte(startAddress & 0xFF)) {
        i2c_stop();
        return -2;
    }

    i2c_start();

    if(i2c_send_byte((slaveAddr << 1) | 0x01)) {
        i2c_stop();
        return -3;
    }

    for(i = 0; i < nMemAddressRead; i++) {
        rData[i] = i2c_receive_byte(1) << 8;
        rData[i] |= i2c_receive_byte(i == nMemAddressRead - 1 ? 0 : 1);
    }

    i2c_stop();
    return 0;
}

int MLX90642_Config(uint8_t slaveAddr, uint16_t writeAddress, uint16_t wData)
{
    // Configuration command format: SA(W) + opcode 0x3A2E + address + value
    i2c_start();

    if(i2c_send_byte(slaveAddr << 1)) {
        i2c_stop();
        return -1;
    }

    if(i2c_send_byte(MLX90642_MS_BYTE(MLX90642_CONFIG_OPCODE)) ||
       i2c_send_byte(MLX90642_LS_BYTE(MLX90642_CONFIG_OPCODE)) ||
       i2c_send_byte(writeAddress >> 8) ||
       i2c_send_byte(writeAddress & 0xFF) ||
       i2c_send_byte(wData >> 8) ||
       i2c_send_byte(wData & 0xFF)) {
        i2c_stop();
        return -2;
    }

    i2c_stop();
    return 0;
}

int MLX90642_I2CCmd(uint8_t slaveAddr, uint16_t i2c_cmd)
{
    // Command format: write command value to address 0x0180
    i2c_start();

    if(i2c_send_byte(slaveAddr << 1)) {
        i2c_stop();
        return -1;
    }

    if(i2c_send_byte(MLX90642_MS_BYTE(MLX90642_CMD_OPCODE)) ||
       i2c_send_byte(MLX90642_LS_BYTE(MLX90642_CMD_OPCODE)) ||
       i2c_send_byte(i2c_cmd >> 8) ||
       i2c_send_byte(i2c_cmd & 0xFF)) {
        i2c_stop();
        return -2;
    }

    i2c_stop();
    return 0;
}

int MLX90642_WakeUp(uint8_t slaveAddr)
{
    (void)slaveAddr;

    // Wake-up command format: START + SA(W) + 0x57 + STOP
    i2c_start();
    if(i2c_send_byte(SA_90642_DEFAULT << 1)) {
        i2c_stop();
        return -1;
    }

    if(i2c_send_byte(0x57)) {
        i2c_stop();
        return -2;
    }

    i2c_stop();
    MLX90642_Wait_ms(10);
    return 0;
}

void MLX90642_Wait_ms(uint16_t time_ms)
{
    delay_ms(time_ms);
}
