#include "my_usart1.h"

_USARTx_GetData USARTx_GetData;
_uart_read_set uart_read_set;
uint8_t buffer[2];  // 高字节和低字节
#define CMD_LENGTH 4  
#define RX_BUF_SIZE 256
uint8_t rx_buffer[CMD_LENGTH];
uint8_t rx_index = 0;

void USART3_Init(void)
{
	
	    RCC_APB2PeriphClockCmd(RCC_GPIOx, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_USAR_RxTx, ENABLE);

		GPIO_InitTypeDef GPIO_InitStructure;
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Tx;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIO_Portx,&GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Rx;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIO_Portx,&GPIO_InitStructure);
		
	
		USART_InitTypeDef USART_InitStructure;
		
		USART_InitStructure.USART_BaudRate= USARTx_brr;
		USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_StopBits= USART_StopBits_1;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART_x,&USART_InitStructure);
		
		USART_ITConfig(USART_x,USART_IT_RXNE,ENABLE);
		USART_ITConfig(USART_x,USART_IT_IDLE,ENABLE);
		

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = USARTx_IR;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =2 ;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		USART_Cmd (USART_x,ENABLE);
		USART_ClearFlag(USART_x,USART_FLAG_RXNE);    //清除串口接收标志位
		USART_ClearFlag(USART_x,USART_FLAG_IDLE);    //清除串口空闲标志位
}



void USART_SendByte(uint8_t Byte)
{
	USART_SendData(USART_x,Byte);
	while(USART_GetFlagStatus(USART_x,USART_FLAG_TXE) == RESET);
	
}


void USART_SendString(char * str)
{
		while(*str !='\0')
		{
			USART_SendByte(*str++);
		}

}
//发送缓冲区
void USART_SendBuff(char *str,uint32_t len)
{
	while(len--)
	{
		USART_SendByte(*str++);
	
	}

}

void USART3_IRQHandler(void)
{
    // 接收中断处理
    if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        uint8_t data = USART_ReceiveData(USART3);
        rx_buffer[rx_index++] = data;
        
        // 缓冲区溢出检查
        if(rx_index >= RX_BUF_SIZE)
        {
            rx_index = 0;
            // 可添加错误处理
        }
    }
    
    // 空闲中断处理
    if(USART_GetITStatus(USART3, USART_IT_IDLE) == SET)
    {
        USART_ReceiveData(USART3); // 必须读取DR寄存器
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
        
        // 处理完整帧
        process_command(rx_buffer, rx_index);
        rx_index = 0; // 重置索引
    }
}

void process_command(uint8_t *data, uint16_t len)
{
    if(len < 4) return; // 最小指令长度校验
    
    if(data[0] == 0xA5 && data[1] == 0x25) // 直流无刷电机
    {
        switch(data[3])
        {
            case 0x15: // PWM占空比设置
				uart_read_set.BLDC_duty = data[2]*0.01;
				uart_read_set.set_state = 1;
                printf("占空比已设置为%d\r\n", data[2]);
                break;
            case 0x25: // 频率设置
				uart_read_set.BLDC_fre = data[2];
				uart_read_set.set_state = 1;
                printf("PWM频率已设置为%d\r\n", data[2]);
                break;
            case 0x35: // 电压设置
				uart_read_set.BLDC_v = data[2];
				uart_read_set.set_state = 1;
                printf("PWM电压已设置为%d\r\n", data[2]);
                break;
			case 0x45: // 启停设置
				if(data[2] == 0x15)
				{
					uart_read_set.BLDC_ready = 1;
					uart_read_set.set_state = 1;
					printf("电机开始转动\r\n");
				}
				else if(data[2] == 0x25)
				{
					uart_read_set.BLDC_ready = 2;
					uart_read_set.set_state = 1;
					printf("电机停止转动转动\r\n");
				}
                break;
        }
    }
	else if(data[0] == 0xA5 && data[1] == 0x35) // 步进电机
    {
        switch(data[3])
        {
            case 0x15: // PWM占空比设置
				uart_read_set.STEP_duty =  data[2]*0.01;
				uart_read_set.set_state = 2;
                printf("占空比已设置为%d\r\n", data[2]);
                break;
            case 0x25: // 频率设置
				uart_read_set.STEP_fre = data[2];
				uart_read_set.set_state = 2;
                printf("步进频率已设置为%d\r\n", data[2]);
                break;
            case 0x35: // 脉冲个数
				uart_read_set.STEP_count = data[2];
				uart_read_set.set_state = 2;
                printf("脉冲个数已设置为%d\r\n", data[2]);
                break;
			case 0x45: // 启停设置
				if(data[2] == 0x15)
				{
					uart_read_set.STEP_ready = 1;
					uart_read_set.set_state = 2;
					printf("电机开始转动\r\n");
				}
				else if(data[2] == 0x25)
				{
					uart_read_set.STEP_ready = 2;
					uart_read_set.set_state = 2;
					printf("电机停止转动转动\r\n");
				}
                break;
			case 0x55: // 正反转设置
				if(data[2] == 0x15)
				{
					uart_read_set.STEP_direction = 1;
					uart_read_set.set_state = 2;
					printf("电机开始正传\r\n");
				}
				else if(data[2] == 0x25)
				{
					uart_read_set.STEP_direction = 2;
					uart_read_set.set_state = 2;
					printf("电机开始反转\r\n");
				}
                break;
			case 0x65: //细分设置
				printf("开始设置细分\r\n");
				uart_read_set.STEP_count_mode = data[2];
				uart_read_set.set_state = 2;
				break;
        }
    }
}

void start_tx(float temperature) {

    uint16_t temp_int = (uint16_t)(temperature * 100);
    buffer[0] = (temp_int >> 8) & 0xFF;
    buffer[1] = temp_int & 0xFF;
	USART_SendByte(buffer[0]);
	USART_SendByte(buffer[1]);
	//printf("0x%X ",buffer[0]);
	//printf("0x%X ",buffer[1]);
}
void shujuti_tx(void)
{
    float sign_byte = 0; // 正负字节（16位）
    start_tx(436.05); // 发送帧头 AA 55

    float temperature;
	float temperature1;
    temperature = SMBus_ReadTemp_TOBJ(); // 读取物体温度
    if (temperature < 0) {
        temperature = -temperature; // 转换为正数
    }
    start_tx(temperature); // 发送物体温度

    temperature1 = SMBus_ReadTemp_TA(); // 读取环境温度
    if (temperature1 < 0) {
        temperature1 = -temperature1; // 转换为正数
    }
    start_tx(temperature1); // 发送环境温度
    // 根据物体温度和环境温度的正负设置正负字节
    if (temperature < 0 && temperature1< 0) {
        sign_byte = 327.69; 
    } else if (temperature > 0 && temperature1 < 0) {
        sign_byte = 1; 
    } else if (temperature > 0 && temperature1 > 0) {
        sign_byte = 0; //F0CC
    } else if (temperature < 0 &&temperature1 > 0) {
        sign_byte = 327.68; 
    }

    start_tx(sign_byte); // 发送正负字节
    start_tx(436.90); // 发送帧尾 AA AA

}
