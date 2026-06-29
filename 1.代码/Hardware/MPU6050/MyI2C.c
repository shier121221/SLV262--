#include "stm32f10x.h"
#include "MyI2C.h"

#define I2C_TIMEOUT      10000U
#define I2C_CLOCK_SPEED  50000U

static uint8_t MyI2C_WaitEvent(uint32_t event)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (I2C_CheckEvent(I2C2, event) != SUCCESS)
    {
        if (timeout-- == 0)
        {
            return MYI2C_ERROR;
        }
    }

    return MYI2C_OK;
}

static uint8_t MyI2C_WaitFlag(FlagStatus status, uint32_t flag)
{
    uint32_t timeout = I2C_TIMEOUT;

    while (I2C_GetFlagStatus(I2C2, flag) != status)
    {
        if (timeout-- == 0)
        {
            return MYI2C_ERROR;
        }
    }

    return MYI2C_OK;
}

static void MyI2C_Stop(void)
{
    I2C_GenerateSTOP(I2C2, ENABLE);
}

static void MyI2C_Delay(void)
{
    volatile uint16_t i;

    for (i = 0; i < 200; i++)
    {
    }
}

static void MyI2C_RecoverBus(void)
{
    uint8_t i;
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_10 | GPIO_Pin_11);
    MyI2C_Delay();

    for (i = 0; i < 9; i++)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_10);
        MyI2C_Delay();
        GPIO_SetBits(GPIOB, GPIO_Pin_10);
        MyI2C_Delay();
    }

    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    MyI2C_Delay();
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    MyI2C_Delay();
    GPIO_SetBits(GPIOB, GPIO_Pin_11);
    MyI2C_Delay();
}

void MyI2C_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    I2C_Cmd(I2C2, DISABLE);
    I2C_DeInit(I2C2);
    MyI2C_RecoverBus();

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_CLOCK_SPEED;
    I2C_Init(I2C2, &I2C_InitStructure);

    I2C_Cmd(I2C2, ENABLE);
}

uint8_t MyI2C_WriteReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t Data)
{
    if (MyI2C_WaitFlag(RESET, I2C_FLAG_BUSY) != MYI2C_OK) return MYI2C_ERROR;

    I2C_GenerateSTART(I2C2, ENABLE);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != MYI2C_OK) goto error;

    I2C_Send7bitAddress(I2C2, DevAddress, I2C_Direction_Transmitter);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != MYI2C_OK) goto error;

    I2C_SendData(I2C2, RegAddress);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != MYI2C_OK) goto error;

    I2C_SendData(I2C2, Data);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != MYI2C_OK) goto error;

    MyI2C_Stop();
    return MYI2C_OK;

error:
    MyI2C_Stop();
    return MYI2C_ERROR;
}

uint8_t MyI2C_ReadReg(uint8_t DevAddress, uint8_t RegAddress, uint8_t *Data)
{
    return MyI2C_ReadRegs(DevAddress, RegAddress, Data, 1);
}

uint8_t MyI2C_ReadRegs(uint8_t DevAddress, uint8_t RegAddress, uint8_t *Data, uint16_t Length)
{
    uint16_t i;

    if (Data == 0 || Length == 0) return MYI2C_ERROR;
    if (MyI2C_WaitFlag(RESET, I2C_FLAG_BUSY) != MYI2C_OK) return MYI2C_ERROR;

    I2C_AcknowledgeConfig(I2C2, ENABLE);

    I2C_GenerateSTART(I2C2, ENABLE);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != MYI2C_OK) goto error;

    I2C_Send7bitAddress(I2C2, DevAddress, I2C_Direction_Transmitter);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != MYI2C_OK) goto error;

    I2C_SendData(I2C2, RegAddress);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != MYI2C_OK) goto error;

    I2C_GenerateSTART(I2C2, ENABLE);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != MYI2C_OK) goto error;

    I2C_Send7bitAddress(I2C2, DevAddress, I2C_Direction_Receiver);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != MYI2C_OK) goto error;

    for (i = 0; i < Length; i++)
    {
        if (i == Length - 1)
        {
            I2C_AcknowledgeConfig(I2C2, DISABLE);
            MyI2C_Stop();
        }

        if (MyI2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) != MYI2C_OK) goto error;
        Data[i] = I2C_ReceiveData(I2C2);
    }

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return MYI2C_OK;

error:
    MyI2C_Stop();
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return MYI2C_ERROR;
}

uint8_t MyI2C_WriteCommand(uint8_t DevAddress, uint16_t Command)
{
    if (MyI2C_WaitFlag(RESET, I2C_FLAG_BUSY) != MYI2C_OK) return MYI2C_ERROR;

    I2C_GenerateSTART(I2C2, ENABLE);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != MYI2C_OK) goto error;

    I2C_Send7bitAddress(I2C2, DevAddress, I2C_Direction_Transmitter);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != MYI2C_OK) goto error;

    I2C_SendData(I2C2, (uint8_t)(Command >> 8));
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != MYI2C_OK) goto error;

    I2C_SendData(I2C2, (uint8_t)Command);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != MYI2C_OK) goto error;

    MyI2C_Stop();
    return MYI2C_OK;

error:
    MyI2C_Stop();
    return MYI2C_ERROR;
}

uint8_t MyI2C_ReadBytes(uint8_t DevAddress, uint8_t *Data, uint16_t Length)
{
    uint16_t i;

    if (Data == 0 || Length == 0) return MYI2C_ERROR;
    if (MyI2C_WaitFlag(RESET, I2C_FLAG_BUSY) != MYI2C_OK) return MYI2C_ERROR;

    I2C_AcknowledgeConfig(I2C2, ENABLE);

    I2C_GenerateSTART(I2C2, ENABLE);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != MYI2C_OK) goto error;

    I2C_Send7bitAddress(I2C2, DevAddress, I2C_Direction_Receiver);
    if (MyI2C_WaitEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != MYI2C_OK) goto error;

    for (i = 0; i < Length; i++)
    {
        if (i == Length - 1)
        {
            I2C_AcknowledgeConfig(I2C2, DISABLE);
            MyI2C_Stop();
        }

        if (MyI2C_WaitEvent(I2C_EVENT_MASTER_BYTE_RECEIVED) != MYI2C_OK) goto error;
        Data[i] = I2C_ReceiveData(I2C2);
    }

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return MYI2C_OK;

error:
    MyI2C_Stop();
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return MYI2C_ERROR;
}
