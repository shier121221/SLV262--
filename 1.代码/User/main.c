#include "stm32f10x.h"                  
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Encoder.h"
#include "Encoder_Soft.h"
#include "LED.h"
#include "motor.h"
#include "Serial.h"
#include "MPU6050.h"
#include "PID.h"
#include "SHT31.h"
#include <string.h>
#include <stdio.h>

#define OLED_ENABLE 0
#define MOTOR_ENCODER_TEST 0
#define SPEED_LOOP_TEST 0
#define TRACK_MODULE_TEST 1
#define TEST_MOTOR_L_PWM 300
#define TEST_MOTOR_R_PWM 500
#define TEST_PRINT_INTERVAL_MS 100
#define SPEED_TEST_TARGET_L 20.0f
#define SPEED_TEST_TARGET_R 20.0f
#define SPEED_TEST_PERIOD_MS 10
#define SPEED_TEST_PRINT_MS 100
#define BAT_ADC_CHANNEL ADC_Channel_4
#define BAT_ADC_SAMPLE_COUNT 16
#define BAT_ADC_REF_VOLTAGE 3.3f
#define BAT_DIVIDER_RATIO ((10.0f + 22.0f) / 10.0f)
#define BAT_CALIBRATION_FACTOR 0.9857f
#define BAT_EMPTY_VOLTAGE 6.6f
#define BAT_FULL_VOLTAGE 8.4f
#define BAT_PRINT_INTERVAL_MS 1000
// 编码器原始数据
int16_t SpeedL;			
int16_t SpeedR;			
float SpeedL_cms;		
float SpeedR_cms;		

// MPU6050数据
int16_t AX, AY, AZ;		
int16_t GX, GY, GZ;

float SHT31_Temperature = 0.0f;
float SHT31_Humidity = 0.0f;
uint8_t SHT31_Ready = 0;
float AvgSpeed = 0.0f;
float AvgDistance = 0.0f;
uint8_t Track_Digital[8] = {0};
uint16_t Track_Analog[8] = {0};
uint16_t Battery_ADC_Raw = 0;
float Battery_Voltage = 0.0f;
uint8_t Battery_Percent = 0;

// 偏航角相关变量
float YawAngle = 0.0f;           // 当前偏航角, 度
float GyroZ_Offset = 0.0f;       // 陀螺仪Z轴零偏
#define GYRO_Z_DEADZONE 0.5f      // 角速度死区(°/s), 低于此值视为静止
#define GYRO_SENSITIVITY 16.4f    // ±2000°/s量程下灵敏度: 16.4 LSB/(°/s)		

// 速度环PID结构体(内环)
pid_type_def PID_SpeedL;	
pid_type_def PID_SpeedR;	

// 位置环PID结构体(外环)
pid_type_def PID_PositionL;
pid_type_def PID_PositionR;

pid_type_def PID_YAW;

float Target_YAW = 0.0f;         // 角度环PID输出(差速速度)
float turn_angle = 0.0f;         // 目标偏航角, 绝对角度, 度
uint8_t TurnMode = 0;            // 转向模式标志: 0=直线模式, 1=转向模式
#define YAW_DEADZONE 1.0f        // 角度环死区, 度
#define YAW_MIN_SPEED 3.0f       // 角度环最小差速速度(cm/s), 低于此值电机驱动不稳

// 位置相关变量
float PositionL = 0;		// 左轮累计位置(cm)
float PositionR = 0;		// 右轮累计位置(cm)
float TargetPositionL = 0;	// 左轮目标位置(cm)
float TargetPositionR = 0;	// 右轮目标位置(cm)

// 速度环控制参数(内环)
float TargetSpeedL = 0.0f;	// 左电机目标速度(cm/s) - 由位置环输出
float TargetSpeedR = 0.0f;	// 右电机目标速度(cm/s) - 由位置环输出
float OutputL = 0;       // 速度环输出(PWM)
float OutputR = 0;       // 速度环输出(PWM)

// 速度环PID参数(内环, 增量式)
PID_Param_t PID_Param_SpeedL = {1.0f, 0.25f, 0.02f};
PID_Param_t PID_Param_SpeedR = {1.0f, 0.25f, 0.02f};

// 位置环PID参数(外环, 位置式, 建议只用P或PI)
PID_Param_t PID_Param_PositionL = {1.5f, 0.02f, 0.0f};
PID_Param_t PID_Param_PositionR = {1.5f, 0.02f, 0.0f};

PID_Param_t PID_Param_YAW = {0.25f, 0.0f, 0.5f};
// 位置环死区(cm), 误差小于此值时认为到达目标
#define POSITION_DEADZONE 0.5f

// 小车轮距(两轮中心间距,单位cm),根据实际车体修改
#define WHEEL_BASE 26.8f
#define WHEEL_DIAMETER_CM 8.5f
#define ENCODER_COUNTS_PER_REV 1040.0f

// PWM限幅最小值, 太小了驱动不了电机
#define PWM_DEADZONE 50

// 记录上一次的目标速度方向, 用于检测方向变化
float LastTargetSpeedL = 0;
float LastTargetSpeedR = 0;  

#define FILTER_QUEUE_LEN 20  // 队列长度
// 速度队列(存储最近若干次的原始速度值)
float SpeedL_Queue[FILTER_QUEUE_LEN] = {0};
float SpeedR_Queue[FILTER_QUEUE_LEN] = {0};
uint8_t Queue_Index = 0;     // 当前写入索引
uint8_t Queue_Count = 0;     // 队列内有效数据个数
float SpeedL_Sum = 0;        // 左队列总和
float SpeedR_Sum = 0;        // 右队列总和
float SpeedL_Filtered = 0;   // 滤波后左速度
float SpeedR_Filtered = 0;   // 滤波后右速度

uint8_t T_10ms = 0;

// 数据发送计数器(每500ms发送一次数据)
uint16_t SendCounter = 0;
#define SEND_INTERVAL 50  // 50*10ms=500ms发送一次

// 数据发送标志位
uint8_t SendDataFlag = 0;

/**
 * 脉冲转速度（恢复正确系数）
 */
float Pulse_To_Speed(int16_t pulse)
{
	return (float)pulse * (3.14159265f * WHEEL_DIAMETER_CM / ENCODER_COUNTS_PER_REV / 0.01f);
}

float Pulse_To_Speed_By_Dt(int16_t pulse, float dt)
{
    return (float)pulse * (3.14159265f * WHEEL_DIAMETER_CM / ENCODER_COUNTS_PER_REV / dt);
}

void Motor_Encoder_TestLoop(void)
{
    int16_t pulseL;
    int16_t pulseR;
    float speedL;
    float speedR;
    const float dt = (float)TEST_PRINT_INTERVAL_MS / 1000.0f;

    MotorL_SetSpeed(TEST_MOTOR_L_PWM);
    MotorR_SetSpeed(TEST_MOTOR_R_PWM);

    while (1)
    {
        Delay_ms(TEST_PRINT_INTERVAL_MS);

        pulseL = -Encoder_Soft_Get();
        pulseR = Encoder_TIM3_Get();
        speedL = Pulse_To_Speed_By_Dt(pulseL, dt);
        speedR = Pulse_To_Speed_By_Dt(pulseR, dt);

        Serial1_Printf("CmdL:%d,SpdL:%.2f,PulseL:%d,CmdR:%d,SpdR:%.2f,PulseR:%d\r\n",
                       TEST_MOTOR_L_PWM, speedL, pulseL,
                       TEST_MOTOR_R_PWM, speedR, pulseR);
    }
}

void Speed_Loop_TestLoop(void)
{
    int16_t pulseL;
    int16_t pulseR;
    int16_t pwmL;
    int16_t pwmR;
    uint16_t printCounter = 0;
    const float dt = (float)SPEED_TEST_PERIOD_MS / 1000.0f;
    const uint16_t printTicks = SPEED_TEST_PRINT_MS / SPEED_TEST_PERIOD_MS;
    fp32 param_SpeedL[3] = {PID_Param_SpeedL.Kp, PID_Param_SpeedL.Ki, PID_Param_SpeedL.Kd};
    fp32 param_SpeedR[3] = {PID_Param_SpeedR.Kp, PID_Param_SpeedR.Ki, PID_Param_SpeedR.Kd};

    PID_init(&PID_SpeedL, PID_DELTA, param_SpeedL, 1000.0f, 100.0f);
    PID_init(&PID_SpeedR, PID_DELTA, param_SpeedR, 1000.0f, 100.0f);
    PID_clear(&PID_SpeedL);
    PID_clear(&PID_SpeedR);

    Serial1_Printf("Speed loop test start\r\n");

    while (1)
    {
        Delay_ms(SPEED_TEST_PERIOD_MS);

        pulseL = -Encoder_Soft_Get();
        pulseR = Encoder_TIM3_Get();
        SpeedL_cms = Pulse_To_Speed_By_Dt(pulseL, dt);
        SpeedR_cms = Pulse_To_Speed_By_Dt(pulseR, dt);

        PID_set_param_s(&PID_SpeedL, &PID_Param_SpeedL);
        PID_set_param_s(&PID_SpeedR, &PID_Param_SpeedR);

        OutputL = PID_calc(&PID_SpeedL, SpeedL_cms, SPEED_TEST_TARGET_L);
        OutputR = PID_calc(&PID_SpeedR, SpeedR_cms, SPEED_TEST_TARGET_R);

        pwmL = (int16_t)OutputL;
        pwmR = (int16_t)OutputR;

        if (SPEED_TEST_TARGET_L == 0.0f)
        {
            pwmL = 0;
            PID_clear(&PID_SpeedL);
        }
        else if (pwmL > 0 && pwmL < PWM_DEADZONE) pwmL = PWM_DEADZONE;
        else if (pwmL < 0 && pwmL > -PWM_DEADZONE) pwmL = -PWM_DEADZONE;

        if (SPEED_TEST_TARGET_R == 0.0f)
        {
            pwmR = 0;
            PID_clear(&PID_SpeedR);
        }
        else if (pwmR > 0 && pwmR < PWM_DEADZONE) pwmR = PWM_DEADZONE;
        else if (pwmR < 0 && pwmR > -PWM_DEADZONE) pwmR = -PWM_DEADZONE;

        MotorL_SetSpeed(pwmL);
        MotorR_SetSpeed(pwmR);

        printCounter++;
        if (printCounter >= printTicks)
        {
            printCounter = 0;
            Serial1_Printf("TgtL:%.2f,SpdL:%.2f,PwmL:%d,TgtR:%.2f,SpdR:%.2f,PwmR:%d\r\n",
                           SPEED_TEST_TARGET_L, SpeedL_cms, pwmL,
                           SPEED_TEST_TARGET_R, SpeedR_cms, pwmR);
        }
    }
}

static uint16_t Track_ReadNumberAfterColon(char **cursor)
{
    uint16_t value = 0;
    char *p = strchr(*cursor, ':');

    if (p == 0)
    {
        return 0;
    }

    p++;
    while (*p >= '0' && *p <= '9')
    {
        value = value * 10 + (uint16_t)(*p - '0');
        p++;
    }

    *cursor = p;
    return value;
}

uint8_t Track_ParsePacket(char *packet)
{
    uint8_t i;
    char *p = packet;

    if (packet[0] != '$')
    {
        return 0;
    }

    if (packet[1] == 'D')
    {
        for (i = 0; i < 8; i++)
        {
            Track_Digital[i] = (uint8_t)Track_ReadNumberAfterColon(&p);
        }
        return 1;
    }
    else if (packet[1] == 'A')
    {
        for (i = 0; i < 8; i++)
        {
            Track_Analog[i] = Track_ReadNumberAfterColon(&p);
        }
        return 2;
    }

    return 0;
}

void Battery_ADC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET);
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET);
}

uint16_t Battery_ADC_ReadRaw(void)
{
    ADC_RegularChannelConfig(ADC1, BAT_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

void Battery_Update(void)
{
    uint8_t i;
    uint32_t sum = 0;
    float percent;

    for (i = 0; i < BAT_ADC_SAMPLE_COUNT; i++)
    {
        sum += Battery_ADC_ReadRaw();
    }

    Battery_ADC_Raw = (uint16_t)(sum / BAT_ADC_SAMPLE_COUNT);
    Battery_Voltage = ((float)Battery_ADC_Raw / 4095.0f) * BAT_ADC_REF_VOLTAGE * BAT_DIVIDER_RATIO * BAT_CALIBRATION_FACTOR;

    percent = (Battery_Voltage - BAT_EMPTY_VOLTAGE) * 100.0f / (BAT_FULL_VOLTAGE - BAT_EMPTY_VOLTAGE);
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    Battery_Percent = (uint8_t)(percent + 0.5f);
}

void Track_Module_TestLoop(void)
{
    uint8_t packetType;
    uint16_t batteryCounter = 0;

    Serial1_Printf("Track module test start\r\n");
    Delay_ms(1000);
    Track_RequestDigitalAnalog();
    Serial1_Printf("Send:$0,1,1#\r\n");

    while (1)
    {
        if (Track_RxFlag == 1)
        {
            packetType = Track_ParsePacket(Track_RxPacket);
            Track_RxFlag = 0;

            if (packetType == 1)
            {
                Serial1_Printf("D:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                               Track_Digital[0], Track_Digital[1],
                               Track_Digital[2], Track_Digital[3],
                               Track_Digital[4], Track_Digital[5],
                               Track_Digital[6], Track_Digital[7]);
            }
            else if (packetType == 2)
            {
                Serial1_Printf("A:%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                               Track_Analog[0], Track_Analog[1],
                               Track_Analog[2], Track_Analog[3],
                               Track_Analog[4], Track_Analog[5],
                               Track_Analog[6], Track_Analog[7]);
            }
            else
            {
                Serial1_Printf("TrackRaw:%s\r\n", Track_RxPacket);
            }
        }

        Delay_ms(10);
        batteryCounter += 10;
        if (batteryCounter >= BAT_PRINT_INTERVAL_MS)
        {
            batteryCounter = 0;
            Battery_Update();
            Serial1_Printf("BAT,Raw:%d,V:%.2f,Pct:%d\r\n",
                           Battery_ADC_Raw, Battery_Voltage, Battery_Percent);
        }
    }
}

/**
 * 位置清零函数（用于重新开始测试）
 */
void Position_Reset(void)
{
	PositionL = 0;
	PositionR = 0;
	PID_clear(&PID_PositionL);
	PID_clear(&PID_PositionR);
}

/**
 * 设置目标位置(单位: cm)
 */
void Set_Target_Position(float target_L, float target_R)
{
	TargetPositionL = target_L;
	TargetPositionR = target_R;
}

/**
 * 原地等比差速转向函数
 * 原理: 左右轮以相同速度、相反方向转动, 实现原地旋转
 *       弧长 = 角度(弧度) × 轮距/2
 * 参数 angle: 转向角度(度), 正值右转, 负值左转
 */
void Turn_InPlace(float angle)
{
    // 角度转弧度
	float angle_rad = angle * 3.14159265f / 180.0f;
	
    // 计算每个轮子需要走的弧长(cm)
	float arc_length = angle_rad * (WHEEL_BASE / 2.0f);
	
    // 正角度(右转): 左轮前进, 右轮后退
    // 负角度(左转): 左轮后退, 右轮前进
	TargetPositionL = PositionL + arc_length;
	TargetPositionR = PositionR - arc_length;
}

/**
 * 陀螺仪Z轴零偏校准函数
 * 上电时保持静止, 采样200次求平均值, 约1秒
 */
void GyroZ_Calibrate(void)
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

/**
 * 偏航角更新函数
 * 通过Z轴陀螺仪角速度积分计算偏航角
 * 参数 dt: 采样周期(秒)
 */
void Yaw_Update(float dt)
{
	// 计算角速度(°/s)
	float gyro_z_dps = ((float)GZ - GyroZ_Offset) / GYRO_SENSITIVITY;
	
	// 死区处理:小角速度视为0,减少静止漂移
	if (gyro_z_dps > -GYRO_Z_DEADZONE && gyro_z_dps < GYRO_Z_DEADZONE)
	{
		gyro_z_dps = 0.0f;
	}
	
    // 积分得到偏航角
	YawAngle += gyro_z_dps * dt;
	
    // 限制在 -180°~180° 范围内
	if (YawAngle > 180.0f) YawAngle -= 360.0f;
	if (YawAngle < -180.0f) YawAngle += 360.0f;
}

/**
 * 队列式滑动平均滤波(完善队列未满逻辑)
 * 功能: 1. 队列未满只入队, 按有效个数算均值; 2. 队列已满入队出队, 按队列长度算均值
 */
void Speed_Queue_Filter(float new_L, float new_R)
{
    // ===== 第一步: 处理队列未满的情况 =====
    if (Queue_Count < FILTER_QUEUE_LEN)
    {
        // 队列未满：直接入队（不删除旧数据），有效计数+1
        SpeedL_Queue[Queue_Index] = new_L;
        SpeedR_Queue[Queue_Index] = new_R;
        
        // 累加新数据到总和
        SpeedL_Sum += new_L;
        SpeedR_Sum += new_R;
        
        // 有效数据个数+1
        Queue_Count++;
        
        // 计算均值: 总和 / 有效数据个数, 避免数值稀释
        SpeedL_Filtered = SpeedL_Sum / Queue_Count;
        SpeedR_Filtered = SpeedR_Sum / Queue_Count;
    }
    // ===== 第二步: 队列已满的情况(原FIFO逻辑) =====
    else
    {
        // 1. 删除最早的数据（减去即将被覆盖的旧值）
        SpeedL_Sum -= SpeedL_Queue[Queue_Index];
        SpeedR_Sum -= SpeedR_Queue[Queue_Index];
        
        // 2. 新数据入队(覆盖最早的数据)
        SpeedL_Queue[Queue_Index] = new_L;
        SpeedR_Queue[Queue_Index] = new_R;
        
        // 3. 累加新数据到总和
        SpeedL_Sum += new_L;
        SpeedR_Sum += new_R;
        
        // 4. 计算均值: 总和 / 队列长度(满队列)
        SpeedL_Filtered = SpeedL_Sum / FILTER_QUEUE_LEN;
        SpeedR_Filtered = SpeedR_Sum / FILTER_QUEUE_LEN;
    }
    
    // ===== 第三步：更新写入索引（环形移动） =====
    Queue_Index++;
    if (Queue_Index >= FILTER_QUEUE_LEN)
    {
        Queue_Index = 0;
    }
}

int main(void)

{
    // 模块初始化
#if OLED_ENABLE
	OLED_Init();			
#endif
#if !MOTOR_ENCODER_TEST && !SPEED_LOOP_TEST && !TRACK_MODULE_TEST
	Timer_Init();			
#endif
	Encoder_TIM3_Init();		
	Encoder_Soft_Init();		
//	LED_Init();
	Serial1_Init();			
    Track_USART2_Init();
    Battery_ADC_Init();
	MotorAll_Init();			
#if MOTOR_ENCODER_TEST
    Serial1_Printf("Motor encoder test start\r\n");
    Motor_Encoder_TestLoop();
#endif
#if SPEED_LOOP_TEST
    Speed_Loop_TestLoop();
#endif
#if TRACK_MODULE_TEST
    Track_Module_TestLoop();
#endif
	MPU6050_Init();			
	SHT31_Init();
    Delay_ms(500);
    // 陀螺仪Z轴零偏校准, 上电保持静止约1秒
	GyroZ_Calibrate();
    Delay_ms(500);
    // 速度环PID初始化(内环, 增量式, PWM限幅0-1000)
    fp32 param_SpeedL[3] = {PID_Param_SpeedL.Kp, PID_Param_SpeedL.Ki, PID_Param_SpeedL.Kd};
    fp32 param_SpeedR[3] = {PID_Param_SpeedR.Kp, PID_Param_SpeedR.Ki, PID_Param_SpeedR.Kd};
    PID_init(&PID_SpeedL, PID_DELTA, param_SpeedL, 1000.0f, 100.0f);
    PID_init(&PID_SpeedR, PID_DELTA, param_SpeedR, 1000.0f, 100.0f);
    
    // 位置环PID初始化(外环, 位置式, 速度限幅±30cm/s)
    fp32 param_PositionL[3] = {PID_Param_PositionL.Kp, PID_Param_PositionL.Ki, PID_Param_PositionL.Kd};
    fp32 param_PositionR[3] = {PID_Param_PositionR.Kp, PID_Param_PositionR.Ki, PID_Param_PositionR.Kd};
    PID_init(&PID_PositionL, PID_POSITION, param_PositionL, 30.0f, 10.0f);
    PID_init(&PID_PositionR, PID_POSITION, param_PositionR, 30.0f, 10.0f);
    
    // 角度环PID初始化(位置式, 输出差速速度, 限幅±40cm/s)
    fp32 param_YAW[3] = {PID_Param_YAW.Kp, PID_Param_YAW.Ki, PID_Param_YAW.Kd};
    PID_init(&PID_YAW, PID_POSITION, param_YAW, 30.0f, 10.0f);
	
    // OLED显示初始化
#if OLED_ENABLE
    OLED_ShowString(1, 1, "D:");         // 第1行: 平均行驶距离(cm)
    OLED_ShowString(2, 1, "T:");     // 第2行: 目标位置
    OLED_ShowString(3, 1, "S:");     // 第3行: 实际速度
    OLED_ShowString(4, 1, "V:");     // 第4行: 目标速度+偏航角
    OLED_ShowString(4, 8, "Y:");     // 偏航角标签
#endif
	
	Delay_ms(500);
	
	while (1)
	{
        if (T_10ms == 1)
        {
            // 串口1打印调试数据(实际角度, 目标角度, 角度环PID输入值, KP, KI, KD)
            
            // OLED显示偏航角, 第4行右侧
#if OLED_ENABLE
            OLED_ShowSignedNum(4, 10, (int16_t)YawAngle, 3);
#endif
            
            T_10ms = 0;
        }
		
        // 向上位机发送数据, 每500ms一次
		if (SendDataFlag == 1)
		{
			SendDataFlag = 0;
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            SHT31_Ready = (SHT31_ReadData(&SHT31_Temperature, &SHT31_Humidity) == 0);
			
            // 发送数据格式: 角度, 温度, 湿度, 当前速度
			AvgSpeed = (SpeedL_Filtered + SpeedR_Filtered) / 2.0f;
			AvgDistance = (PositionL + PositionR) / 2.0f;
			
            Serial1_Printf("Angle:%.2f,T:%.2f,H:%.2f,Speed:%.2f\r\n",
                           YawAngle, SHT31_Temperature, SHT31_Humidity, AvgSpeed);
		}
		
		// 处理USART1接收到的命令
		if (Serial1_RxFlag == 1)
		{
			// 解析命令格式: "MOVE,距离"
			if (strncmp(Serial1_RxPacket, "MOVE,", 5) == 0)
			{
				float distance = 0;
				sscanf(Serial1_RxPacket + 5, "%f", &distance);
				
                // 退出转向模式, 进入直线模式
				TurnMode = 0;
				// 设置目标位置(两轮同时前进相同距离)
				TargetPositionL = PositionL + distance;
				TargetPositionR = PositionR + distance;
			}
            // 解析命令格式: "TURN,角度" (正值右转, 负值左转)
			else if (strncmp(Serial1_RxPacket, "TURN,", 5) == 0)
			{
				float angle_offset = 0;
				sscanf(Serial1_RxPacket + 5, "%f", &angle_offset);
                // 目标偏航角 = 当前偏航角 + 输入角度
				turn_angle = YawAngle + angle_offset;
				// 进入转向模式
				TurnMode = 1;
                // 清零角度环PID累加值, 避免上次残留影响
				PID_clear(&PID_YAW);
				PID_clear(&PID_SpeedL);
				PID_clear(&PID_SpeedR);
			}
			else if (strcmp(Serial1_RxPacket, "STOP") == 0)
			{
                // 停止命令: 退出转向模式, 目标位置设为当前位置
				TurnMode = 0;
				TargetPositionL = PositionL;
				TargetPositionR = PositionR;
			}
			else if (strcmp(Serial1_RxPacket, "RESET") == 0)
			{
                // 位置清零, 退出转向模式
				TurnMode = 0;
				Position_Reset();
			}
			
			Serial1_RxFlag = 0;  // 清除接收标志
		}
		

        // 读取MPU6050数据(主循环高频更新, 保证中断使用最新GZ值)
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
		
        // 第1行显示平均行驶距离(cm)
#if OLED_ENABLE
		OLED_ShowSignedNum(1, 3, (int16_t)((PositionL + PositionR) / 2.0f), 5);
        // 第2行显示目标位置(cm)
		OLED_ShowSignedNum(2, 3, (int16_t)TargetPositionL, 5);
		OLED_ShowSignedNum(2, 9, (int16_t)TargetPositionR, 5);
 
        // 第3行显示实际速度(cm/s)
		OLED_ShowSignedNum(3, 3, (int16_t)SpeedL_Filtered, 4);
		OLED_ShowSignedNum(3, 9, (int16_t)SpeedR_Filtered, 4);
 
        // 第4行左侧显示目标速度(cm/s); 右侧偏航角在T_10ms中更新
		OLED_ShowSignedNum(4, 3, (int16_t)TargetSpeedL, 4);
#endif
	}
}

/**
 * TIM1中断: 10ms一次, 双闭环PID控制
 * 控制流程: 位置累加 -> 位置环PID -> 目标速度 -> 速度滤波 -> 速度环PID -> PWM输出
 */
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		// ========== 第1步: 读取编码器原始脉冲, 计算速度 ==========
		SpeedL = -Encoder_Soft_Get();		
		SpeedR = Encoder_TIM3_Get();		
		SpeedL_cms = Pulse_To_Speed(SpeedL);
		SpeedR_cms = Pulse_To_Speed(SpeedR);
		
        // ========== 第2步: 累加位置(速度×时间, 10ms=0.01s) ==========
		PositionL += SpeedL_cms * 0.01f;
        PositionR += SpeedR_cms * 0.01f;
		
        // ========== 第2.5步: 更新偏航角, 在控制计算之前确保使用最新角度 ==========
		Yaw_Update(0.01f);
		
        // ========== 第3步: 根据模式选择控制策略 ==========
		if (TurnMode == 1)
		{
            // ===== 转向模式: 角度环PID -> 差速速度 -> 速度环 =====
			PID_set_param_s(&PID_YAW, &PID_Param_YAW);
			
			// 计算角度误差
			float YAW_Error = turn_angle - YawAngle;
			
            // 角度死区判断: 误差小于死区时停止转向
			if (YAW_Error > -YAW_DEADZONE && YAW_Error < YAW_DEADZONE)
			{
//				Target_YAW = 0;
				TargetSpeedL = 0;
				TargetSpeedR = 0;
				PID_clear(&PID_YAW);
				PID_clear(&PID_SpeedL);
				PID_clear(&PID_SpeedR);

			}
			else
			{
				// 角度环PID输出差速速度(cm/s)
				Target_YAW = PID_calc(&PID_YAW, YawAngle, turn_angle);
                // 最小速度钳位: 保证电机有足够扭矩转动
				if (Target_YAW > 0 && Target_YAW < YAW_MIN_SPEED) Target_YAW = YAW_MIN_SPEED;
				else if (Target_YAW < 0 && Target_YAW > -YAW_MIN_SPEED) Target_YAW = -YAW_MIN_SPEED;
                // 左轮正转, 右轮反转 -> 原地等比差速转向
				TargetSpeedL = -Target_YAW;
				TargetSpeedR = +Target_YAW;
			}
		}
		else
		{
            // ===== 直线模式: 使用位置环控制目标位置 =====
			TargetSpeedL = 0;
			TargetSpeedR = 0;
			
            /* === 位置环控制代码 === */
			PID_set_param_s(&PID_PositionL, &PID_Param_PositionL);
			PID_set_param_s(&PID_PositionR, &PID_Param_PositionR);
			
			float errorL = TargetPositionL - PositionL;
			float errorR = TargetPositionR - PositionR;
			
			if (errorL > -POSITION_DEADZONE && errorL < POSITION_DEADZONE)
			{
				TargetSpeedL = 0;
				PID_clear(&PID_SpeedL);
			}
			else
			{
				TargetSpeedL = PID_calc(&PID_PositionL, PositionL, TargetPositionL);
				
				if ((LastTargetSpeedL >= 0 && TargetSpeedL < 0) || (LastTargetSpeedL < 0 && TargetSpeedL >= 0))
				{
					PID_clear(&PID_SpeedL);
				}
				LastTargetSpeedL = TargetSpeedL;
			}
			
			if (errorR > -POSITION_DEADZONE && errorR < POSITION_DEADZONE)
			{
				TargetSpeedR = 0;
				PID_clear(&PID_SpeedR);
			}
			else
			{
				TargetSpeedR = PID_calc(&PID_PositionR, PositionR, TargetPositionR);
				
				if ((LastTargetSpeedR >= 0 && TargetSpeedR < 0) || (LastTargetSpeedR < 0 && TargetSpeedR >= 0))
				{
					PID_clear(&PID_SpeedR);
				}
				LastTargetSpeedR = TargetSpeedR;
			}

		}
		
        // ========== 第4步: 速度队列式滤波 ==========
		Speed_Queue_Filter(SpeedL_cms, SpeedR_cms);
		
        // ========== 第5步: 速度环PID计算(内环, 输出PWM) ==========
        PID_set_param_s(&PID_SpeedL, &PID_Param_SpeedL);
		PID_set_param_s(&PID_SpeedR, &PID_Param_SpeedR);
        
		OutputL = PID_calc(&PID_SpeedL, SpeedL_Filtered, TargetSpeedL);
		OutputR = PID_calc(&PID_SpeedR, SpeedR_Filtered, TargetSpeedR);
		
        // ========== 第6步: 电机PWM输出(限幅50-1000, 避免低速扭矩不足) ==========
        // 对PWM输出进行最小值限幅
		int16_t pwmL = (int16_t)OutputL;
		int16_t pwmR = (int16_t)OutputR;
		
        // 如果PWM绝对值小于死区, 则设为死区值; 否则保持计算值
		if (pwmL > 0 && pwmL < PWM_DEADZONE) pwmL = PWM_DEADZONE;
		else if (pwmL < 0 && pwmL > -PWM_DEADZONE) pwmL = -PWM_DEADZONE;
		
		if (pwmR > 0 && pwmR < PWM_DEADZONE) pwmR = PWM_DEADZONE;
		else if (pwmR < 0 && pwmR > -PWM_DEADZONE) pwmR = -PWM_DEADZONE;
		
		MotorL_SetSpeed(pwmL);
		MotorR_SetSpeed(pwmR);
		
        T_10ms = 1;
		
        // ========== 第7步: 数据发送计数(每500ms设置一次标志位) ==========
		SendCounter++;
		if (SendCounter >= SEND_INTERVAL)
		{
			SendCounter = 0;
            SendDataFlag = 1;  // 设置发送标志位, 在主循环中发送
		}
		
		// 清除中断标志
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

