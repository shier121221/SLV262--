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

#define MLX90642_ADDR     0x33 << 1 // 7位地址左移1位
uint16_t rawData[769]; // 768像素+1校验
float tempMap[768];

int main(void)
{
			uint16_t  i = 0;
			uint16_t  row = 0;
			uint16_t  col = 0;
	
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
			delay_init();
			uart_init(115200); 											//打印调试串口
			i2c_config();														//IIC初始化配置
			
			// 初始化阶段
			MLX90642_WakeUp(MLX90642_ADDR);
			MLX90642_Init(MLX90642_ADDR);
				
			// 设置测量模式（文档2示例）
			MLX90642_SetMeasMode(MLX90642_ADDR, MLX90642_CONT_MEAS_MODE);
			MLX90642_SetRefreshRate(MLX90642_ADDR, MLX90642_REF_RATE_8HZ);

			printf("Litus_MLX90642初始化成功!!!\r\n");
			while (1)
			{
						// 等待新数据就绪
						int status;
						do {
								status = MLX90642_IsReadWindowOpen(MLX90642_ADDR);
						} while(status == MLX90642_NO);
						
						// 读取原始数据（文档2说明）
						if(MLX90642_GetImage(MLX90642_ADDR, rawData) < 0) {
								printf("Read Error!\r\n");
								continue;
						}
						
						// 数据转换（根据MLX90642数据手册公式）
						for(i=0; i<768; i++) {
								tempMap[i] = rawData[i]*0.02 ; // 转换为摄氏度
						}
						
						// 矩阵输出（32x24排列）
						printf("----- Temperature Matrix -----\r\n");
						for(row=0; row<24; row++) {
								for(col=0; col<32; col++) {
										printf("%5.2f℃  ", tempMap[row*32 + col]);
								}
								printf("\r\n");
						}
			}
} 
