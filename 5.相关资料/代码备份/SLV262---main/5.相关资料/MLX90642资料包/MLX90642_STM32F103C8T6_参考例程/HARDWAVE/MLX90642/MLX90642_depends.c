#include "MLX90642_depends.h"
#include "stm32f10x.h"
#include "myiic.h"

int MLX90642_I2CRead(uint8_t slaveAddr, uint16_t startAddress, 
                    uint16_t nMemAddressRead, uint16_t *rData)
{
	uint16_t i = 0;
    /* I2C读取协议实现 */
    i2c_start();
    
    // 发送设备地址（写模式）
    if(i2c_send_byte(slaveAddr << 1)) {
        i2c_stop();
        return -1; // 设备无响应
    }
    
    // 发送16位内存地址（高位在前）
    if(i2c_send_byte(startAddress >> 8) || 
       i2c_send_byte(startAddress & 0xFF)) {
        i2c_stop();
        return -2; // 地址发送失败  
    }
    
    // 重复起始条件
    i2c_start();
    
    // 发送设备地址（读模式）
    if(i2c_send_byte((slaveAddr << 1) | 0x01)) {
        i2c_stop();
        return -3; 
    }
    
    // 连续读取数据

    for(i=0; i<nMemAddressRead; i++){
        rData[i] = i2c_receive_byte(1) << 8; // 高字节
        rData[i] |= i2c_receive_byte(i == nMemAddressRead-1 ? 0 : 1); //低字节
        
        // 最后字节发送NACK
        if(i == nMemAddressRead-1)
            i2c_no_ack();
    }
    
    i2c_stop();
    return 0;
}

int MLX90642_Config(uint8_t slaveAddr, uint16_t writeAddress, uint16_t wData)
{
    /* 配置寄存器写入 */
    i2c_start();
    
    // 设备地址+写模式
    if(i2c_send_byte(slaveAddr << 1)) {
        i2c_stop();
        return -1;
    }
    
    // 发送16位地址
    if(i2c_send_byte(writeAddress >> 8) || 
       i2c_send_byte(writeAddress & 0xFF)) {
        i2c_stop();
        return -2;
    }
    
    // 发送16位数据
    if(i2c_send_byte(wData >> 8) || 
       i2c_send_byte(wData & 0xFF)) {
        i2c_stop();
        return -3;
    }
    
    i2c_stop();
    return 0;
}

int MLX90642_I2CCmd(uint8_t slaveAddr, uint16_t i2c_cmd)
{
    /* 特殊命令发送 */
    i2c_start();
    
    // 发送命令格式：设备地址 + 命令字高8位
    if(i2c_send_byte(slaveAddr << 1) || 
       i2c_send_byte(i2c_cmd >> 8)) {
        i2c_stop();
        return -1;
    }
    
    // 命令字低8位
    if(i2c_send_byte(i2c_cmd & 0xFF)) {
        i2c_stop();
        return -2;
    }
    
    i2c_stop();
    return 0;
}

int MLX90642_WakeUp(uint8_t slaveAddr)
{
    /* 唤醒协议实现 */
    // MLX90642需要发送特殊唤醒脉冲
    i2c_start();
    
    // 发送设备地址（不检查ACK）
    i2c_send_byte(slaveAddr << 1); 
    
    // 保持SCL低电平至少33ms（根据手册要求）
    MLX90642_Wait_ms(35); 
    i2c_stop();
    
    return 0;
}
void MLX90642_Wait_ms(uint16_t time_ms)
{
	delay_ms(time_ms);
}
