#include "myiic.h"

/** 
  * @brief  i2c gpio initialization.
  * @param  none.
  * @retval none.
  */
#define frequent 2

void i2c_Wait(int t)
{
    int cnt; 
    while(t--)
    for(cnt=7;cnt>0;cnt--); 
}
void i2c_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // 打开 GPIO 时钟
    RCC_APB2PeriphClockCmd(I2Cx_SCL_GPIO_CLK | I2Cx_SDA_GPIO_CLK, ENABLE);

    // 设置 SCL 引脚为开漏输出，带上拉
    GPIO_InitStructure.GPIO_Pin = I2Cx_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

    // 设置 SDA 引脚为开漏输出，带上拉
    GPIO_InitStructure.GPIO_Pin = I2Cx_SDA_PIN;
    GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);

    // 默认拉高
    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}
/** 
  * @brief  used to set the i2c clock frequency.
  * @param  none.
  * @retval none.
  */
void i2c_delay(void)
{
  delay_us(5);
}
 
/** 
  * @brief  used to generate start conditions.
  * @param  none.
  * @retval none.
  */
void i2c_start(void)
{
  i2c_Wait(frequent);
  
  I2C_SDA_HIGH();
  I2C_SCL_HIGH();
  i2c_Wait(frequent);
  
  I2C_SDA_LOW();
  i2c_Wait(frequent);
  
  I2C_SCL_LOW();
}
 
/** 
  * @brief  used to generate stop conditions.
  * @param  none.
  * @retval none.
  */
void i2c_stop(void)
{
  I2C_SCL_LOW();
  I2C_SDA_LOW();  
  i2c_Wait(frequent);
  
  I2C_SCL_HIGH();
  i2c_Wait(frequent);
  
  I2C_SDA_HIGH();
  i2c_Wait(frequent);
}
 
/** 
  * @brief  used to generate ack conditions.
  * @param  none.
  * @retval none.
  */
void i2c_ack(void)
{
  I2C_SCL_LOW();
  I2C_SDA_LOW();
  i2c_Wait(frequent);
  
  I2C_SCL_HIGH();
  i2c_Wait(frequent);
  
  I2C_SCL_LOW();
}
 
/** 
  * @brief  used to generate nack conditions.
  * @param  none.
  * @retval none.
  */
void i2c_no_ack(void)
{
  I2C_SCL_LOW();
  I2C_SDA_HIGH();
  i2c_Wait(frequent);
  
  I2C_SCL_HIGH();
  i2c_Wait(frequent);
  
  I2C_SCL_LOW();
}
 
/** 
  * @brief  used to wait ack conditions.
  * @param  timeout: poll count while SCL is high.
  * @retval ack receive status.
  *         - 1: no ack received.
  *         - 0: ack received.
  */
uint8_t I2CReceiveAck(uint8_t timeout)
{
  I2C_SCL_LOW();
  I2C_SDA_HIGH();
  i2c_Wait(frequent);

  // ACK 必须在 SCL 高电平期间采样
  I2C_SCL_HIGH();
  i2c_Wait(frequent);

  while(timeout)
  {
    if (I2C_SDA_READ() == 0)
    {
      I2C_SCL_LOW();
      i2c_Wait(frequent);
      return 0;
    }

    i2c_Wait(frequent);
    timeout--;
  }

  I2C_SCL_LOW();
  i2c_Wait(frequent);

  return 1;
}
 
/** 
  * @brief  send a byte.
  * @param  data: byte to be transmitted.
  * @retval none.
  */
unsigned char i2c_send_byte(uint8_t data)
{
  unsigned char ack = 1;
  uint8_t i = 8;
  while (i--)
  {
    I2C_SCL_LOW();
    
    if (data & 0x80)
    {
      I2C_SDA_HIGH();    
    }
    else
    {
      I2C_SDA_LOW();    
    }    
    
    i2c_Wait(frequent);
 
    data <<= 1;
    
    I2C_SCL_HIGH();
    i2c_Wait(frequent);
  }
  i2c_Wait(frequent);
  ack = I2CReceiveAck(8);
  I2C_SCL_LOW();
  I2C_SDA_HIGH();  
  return ack;
}
 
/** 
  * @brief  receive a byte.
  * @param  data: byte to be received.
  * @retval none.
  */
uint8_t i2c_receive_byte(unsigned char ack)
{
  uint8_t i = 8;
  uint8_t byte = 0;
 
  I2C_SDA_HIGH();
  
  while (i--) 
  {
    byte <<= 1;
    
    I2C_SCL_LOW();
    i2c_Wait(frequent);
    
    I2C_SCL_HIGH();
    i2c_Wait(frequent);
    
    if (I2C_SDA_READ()) 
    {
      byte |= 0x01;
    }
  }
  
  I2C_SCL_LOW();
  if (!ack)
    i2c_no_ack();//发送nACK
  else
    i2c_ack(); //发送ACK
  return byte;
}

void I2CReadBytes(int nBytes, char *dataP)
{
    char data;
    int i = 0, j = 0;
    for(j=0;j<nBytes;j++)
    {
        i2c_Wait(frequent);
        I2C_SDA_HIGH();  
        
        data = 0;
        for(i=0;i<8;i++){
            i2c_Wait(frequent);
            I2C_SCL_HIGH();
            i2c_Wait(frequent);
            data = data<<1;
            if(I2C_SDA_READ()){
                data = data+1;  
            }
            i2c_Wait(frequent);
            I2C_SCL_LOW();
            i2c_Wait(frequent);
        }  
        
        if(j == (nBytes-1))
        {
            i2c_no_ack();
        }
        else
        {                  
            i2c_ack();
        }
            
        *(dataP+j) = data; 
    }    
    
}
