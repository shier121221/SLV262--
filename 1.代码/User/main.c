#include "stm32f10x.h"                  
#include "Delay.h"
#include "Timer.h"
#include "Encoder.h"
#include "Encoder_Soft.h"
#include "LED.h"
#include "motor.h"
#include "Serial.h"
#include "Protocol.h"
#include "MPU6050.h"
#include "PID.h"
#include "SHT31.h"
#include "MLX90642_App.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define SENSOR_TEST_ENABLE 0
#define MOTOR_ENCODER_TEST 0
#define SPEED_LOOP_TEST 0
#define TRACK_MODULE_TEST 0
#define MLX90642_ENABLE 1
#define TRACK_AUTO_ENABLE 1
#define DEBUG_TEXT_ENABLE 0
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
#define TRACK_REQUEST_INTERVAL_MS 50
#define TRACK_TIMEOUT_MS 300
#define TRACK_BASE_SPEED_CMS 18.0f
#define TRACK_TURN_KP 0.35f
#define TRACK_TURN_MAX_CMS 12.0f
// 缂栫爜鍣ㄥ師濮嬫暟鎹?int16_t SpeedL;			
int16_t SpeedL;			
int16_t SpeedR;			
float SpeedL_cms;		
float SpeedR_cms;		

// MPU6050鏁版嵁
int16_t AX, AY, AZ;		
int16_t GX, GY, GZ;
float GyroZ_Dps = 0.0f;

float SHT31_Temperature = 0.0f;
float SHT31_Humidity = 0.0f;
uint8_t SHT31_Ready = 0;
float AvgSpeed = 0.0f;
float AvgDistance = 0.0f;
uint8_t Track_Digital[8] = {0};
uint16_t Track_Analog[8] = {0};
uint8_t Track_Ready = 0;
uint16_t Track_AgeMs = TRACK_TIMEOUT_MS;
int16_t Track_Error = 0;
float Track_TurnSpeed = 0.0f;
uint8_t Track_Pattern = 0;
uint8_t Track_UseAnalog = 0;
uint16_t Battery_ADC_Raw = 0;
float Battery_Voltage = 0.0f;
uint8_t Battery_Percent = 0;
float MLX90642_TempMap[MLX90642_APP_PIXELS];
uint8_t MLX90642_Ready = 0;

// 鍋忚埅瑙掔浉鍏冲彉閲?
float YawAngle = 0.0f;           // 褰撳墠鍋忚埅瑙? 搴?
float GyroZ_Offset = 0.0f;       // 闄€铻轰华Z杞撮浂鍋?
#define GYRO_Z_DEADZONE 0.5f      // 瑙掗€熷害姝诲尯(掳/s), 浣庝簬姝ゅ€艰涓洪潤姝?
#define GYRO_SENSITIVITY 16.4f    // 卤2000掳/s閲忕▼涓嬬伒鏁忓害: 16.4 LSB/(掳/s)		

// 閫熷害鐜疨ID缁撴瀯浣?鍐呯幆)
pid_type_def PID_SpeedL;	
pid_type_def PID_SpeedR;	

// 浣嶇疆鐜疨ID缁撴瀯浣?澶栫幆)
pid_type_def PID_PositionL;
pid_type_def PID_PositionR;

pid_type_def PID_YAW;

float Target_YAW = 0.0f;         // 瑙掑害鐜疨ID杈撳嚭(宸€熼€熷害)
float turn_angle = 0.0f;         // 鐩爣鍋忚埅瑙? 缁濆瑙掑害, 搴?
uint8_t TurnMode = 0;            // 杞悜妯″紡鏍囧織: 0=鐩寸嚎妯″紡, 1=杞悜妯″紡
#define YAW_DEADZONE 1.0f        // 瑙掑害鐜鍖? 搴?
#define YAW_MIN_SPEED 3.0f       // 瑙掑害鐜渶灏忓樊閫熼€熷害(cm/s), 浣庝簬姝ゅ€肩數鏈洪┍鍔ㄤ笉绋?

// 浣嶇疆鐩稿叧鍙橀噺
float PositionL = 0;		// 宸﹁疆绱浣嶇疆(cm)
float PositionR = 0;		// 鍙宠疆绱浣嶇疆(cm)
float TargetPositionL = 0;	// 宸﹁疆鐩爣浣嶇疆(cm)
float TargetPositionR = 0;	// 鍙宠疆鐩爣浣嶇疆(cm)

// 閫熷害鐜帶鍒跺弬鏁?鍐呯幆)
float TargetSpeedL = 0.0f;	// 宸︾數鏈虹洰鏍囬€熷害(cm/s) - 鐢变綅缃幆杈撳嚭
float TargetSpeedR = 0.0f;	// 鍙崇數鏈虹洰鏍囬€熷害(cm/s) - 鐢变綅缃幆杈撳嚭
float OutputL = 0;       // 閫熷害鐜緭鍑?PWM)
float OutputR = 0;       // 閫熷害鐜緭鍑?PWM)

// 閫熷害鐜疨ID鍙傛暟(鍐呯幆, 澧為噺寮?
PID_Param_t PID_Param_SpeedL = {1.0f, 0.25f, 0.02f};
PID_Param_t PID_Param_SpeedR = {1.0f, 0.25f, 0.02f};

// 浣嶇疆鐜疨ID鍙傛暟(澶栫幆, 浣嶇疆寮? 寤鸿鍙敤P鎴朠I)
PID_Param_t PID_Param_PositionL = {1.5f, 0.02f, 0.0f};
PID_Param_t PID_Param_PositionR = {1.5f, 0.02f, 0.0f};

PID_Param_t PID_Param_YAW = {0.25f, 0.0f, 0.5f};
// 浣嶇疆鐜鍖?cm), 璇樊灏忎簬姝ゅ€兼椂璁や负鍒拌揪鐩爣
#define POSITION_DEADZONE 0.5f

// 灏忚溅杞窛(涓よ疆涓績闂磋窛,鍗曚綅cm),鏍规嵁瀹為檯杞︿綋淇敼
#define WHEEL_BASE 26.8f
#define WHEEL_DIAMETER_CM 8.5f
#define ENCODER_COUNTS_PER_REV 1040.0f

// PWM闄愬箙鏈€灏忓€? 澶皬浜嗛┍鍔ㄤ笉浜嗙數鏈?
#define PWM_DEADZONE 50

// 璁板綍涓婁竴娆＄殑鐩爣閫熷害鏂瑰悜, 鐢ㄤ簬妫€娴嬫柟鍚戝彉鍖?
float LastTargetSpeedL = 0;
float LastTargetSpeedR = 0;  

#define FILTER_QUEUE_LEN 20  // 闃熷垪闀垮害
// 閫熷害闃熷垪(瀛樺偍鏈€杩戣嫢骞叉鐨勫師濮嬮€熷害鍊?
float SpeedL_Queue[FILTER_QUEUE_LEN] = {0};
float SpeedR_Queue[FILTER_QUEUE_LEN] = {0};
uint8_t Queue_Index = 0;     // 褰撳墠鍐欏叆绱㈠紩
uint8_t Queue_Count = 0;     // 闃熷垪鍐呮湁鏁堟暟鎹釜鏁?
float SpeedL_Sum = 0;        // 宸﹂槦鍒楁€诲拰
float SpeedR_Sum = 0;        // 鍙抽槦鍒楁€诲拰
float SpeedL_Filtered = 0;   // 婊ゆ尝鍚庡乏閫熷害
float SpeedR_Filtered = 0;   // 婊ゆ尝鍚庡彸閫熷害

uint8_t T_10ms = 0;

// 鏁版嵁鍙戦€佽鏁板櫒(姣?00ms鍙戦€佷竴娆℃暟鎹?
uint16_t SendCounter = 0;
#define SEND_INTERVAL 50  // 50*10ms=500ms鍙戦€佷竴娆?

// 鏁版嵁鍙戦€佹爣蹇椾綅
uint8_t SendDataFlag = 0;

/**
 * 鑴夊啿杞€熷害锛堟仮澶嶆纭郴鏁帮級
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

int16_t Track_CalcError(void)
{
    static const int8_t weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    int16_t sum = 0;
    uint8_t count = 0;
    uint8_t i;

    Track_Pattern = 0;
    for (i = 0; i < 8; i++)
    {
        Track_Pattern <<= 1;
        Track_Pattern |= (Track_Digital[i] & 0x01);

        if (Track_Digital[i] == 0)
        {
            sum += weights[i];
            count++;
        }
    }

    if (count == 0)
    {
        return Track_Error;
    }

    return sum / count;
}

uint8_t Track_CalcAnalogError(int16_t *error)
{
    static const int8_t weights[8] = {-35, -25, -15, -5, 5, 15, 25, 35};
    uint16_t minValue = Track_Analog[0];
    uint16_t maxValue = Track_Analog[0];
    uint32_t sumDark = 0;
    uint32_t sumLight = 0;
    int32_t weightedDark = 0;
    int32_t weightedLight = 0;
    uint16_t darkStrength;
    uint16_t lightStrength;
    uint8_t i;

    for (i = 1; i < 8; i++)
    {
        if (Track_Analog[i] < minValue) minValue = Track_Analog[i];
        if (Track_Analog[i] > maxValue) maxValue = Track_Analog[i];
    }

    if ((maxValue - minValue) < 20)
    {
        return 0;
    }

    for (i = 0; i < 8; i++)
    {
        darkStrength = maxValue - Track_Analog[i];
        lightStrength = Track_Analog[i] - minValue;

        sumDark += darkStrength;
        sumLight += lightStrength;
        weightedDark += (int32_t)darkStrength * weights[i];
        weightedLight += (int32_t)lightStrength * weights[i];
    }

    if (sumDark >= sumLight && sumDark > 0)
    {
        *error = (int16_t)(weightedDark / (int32_t)sumDark);
    }
    else if (sumLight > 0)
    {
        *error = (int16_t)(weightedLight / (int32_t)sumLight);
    }
    else
    {
        return 0;
    }

    return 1;
}

void Track_Service_10ms(void)
{
    static uint16_t requestCounter = TRACK_REQUEST_INTERVAL_MS;
    uint8_t packetType;

    if (Track_AgeMs < 1000)
    {
        Track_AgeMs += 10;
    }

    requestCounter += 10;
    if (requestCounter >= TRACK_REQUEST_INTERVAL_MS)
    {
        requestCounter = 0;
        Track_RequestDigitalAnalog();
    }

    if (Track_RxFlag == 1)
    {
        packetType = Track_ParsePacket(Track_RxPacket);
        Track_RxFlag = 0;

        if (packetType == 1)
        {
            if (Track_UseAnalog == 0)
            {
                Track_Error = Track_CalcError();
                Track_AgeMs = 0;
                Track_Ready = 1;
            }
        }
        else if (packetType == 2)
        {
            if (Track_CalcAnalogError(&Track_Error))
            {
                Track_AgeMs = 0;
                Track_Ready = 1;
                Track_UseAnalog = 1;
            }
            else if (Track_Ready)
            {
                Track_AgeMs = 0;
            }
        }
    }

    if (Track_AgeMs >= TRACK_TIMEOUT_MS)
    {
        Track_Ready = 0;
        Track_UseAnalog = 0;
        Track_TurnSpeed = 0.0f;
    }
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

static int16_t Float_To_Centi(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value * 100.0f + 0.5f);
    }

    return (int16_t)(value * 100.0f - 0.5f);
}

static void Telemetry_SendFrameBlocking(uint8_t msgId, const uint8_t *payload, uint8_t len)
{
    uint32_t retry = 0;

    while (Protocol_SendFrame(msgId, payload, len) != 0 && retry < 100000)
    {
        retry++;
    }
}

void Telemetry_SendBasic(void)
{
    CarStatusPayload_t car;
    EnvBatteryPayload_t env;
    TrackStatusPayload_t track;
    uint8_t i;

    car.tick_ms = SendCounter;
    car.state = 0;
    car.fault = 0;
    car.lap_count = 0;
    car.battery_percent = Battery_Percent;
    car.speed_l_cms = (int16_t)SpeedL_Filtered;
    car.speed_r_cms = (int16_t)SpeedR_Filtered;
    car.yaw_cdeg = Float_To_Centi(YawAngle);
    Telemetry_SendFrameBlocking(PROTOCOL_MSG_CAR_STATUS, (uint8_t *)&car, sizeof(car));

    env.tick_ms = SendCounter;
    env.temp_centi = Float_To_Centi(SHT31_Temperature);
    env.humi_centi = Float_To_Centi(SHT31_Humidity);
    env.battery_mv = (uint16_t)(Battery_Voltage * 1000.0f + 0.5f);
    env.battery_percent = Battery_Percent;
    env.power_state = SHT31_Ready ? 0 : 1;
    Telemetry_SendFrameBlocking(PROTOCOL_MSG_ENV_BATTERY, (uint8_t *)&env, sizeof(env));

    track.tick_ms = SendCounter;
    track.track_error = Track_Error;
    track.track_ready = Track_Ready;
    track.use_analog = Track_UseAnalog;
    for (i = 0; i < 8; i++)
    {
        track.analog[i] = Track_Analog[i];
    }
    Telemetry_SendFrameBlocking(PROTOCOL_MSG_TRACK_STATUS, (uint8_t *)&track, sizeof(track));
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
 * 浣嶇疆娓呴浂鍑芥暟锛堢敤浜庨噸鏂板紑濮嬫祴璇曪級
 */
void Position_Reset(void)
{
	PositionL = 0;
	PositionR = 0;
	PID_clear(&PID_PositionL);
	PID_clear(&PID_PositionR);
}

/**
 * 璁剧疆鐩爣浣嶇疆(鍗曚綅: cm)
 */
void Set_Target_Position(float target_L, float target_R)
{
	TargetPositionL = target_L;
	TargetPositionR = target_R;
}

/**
 * 鍘熷湴绛夋瘮宸€熻浆鍚戝嚱鏁?
 * 鍘熺悊: 宸﹀彸杞互鐩稿悓閫熷害銆佺浉鍙嶆柟鍚戣浆鍔? 瀹炵幇鍘熷湴鏃嬭浆
 *       寮ч暱 = 瑙掑害(寮у害) 脳 杞窛/2
 * 鍙傛暟 angle: 杞悜瑙掑害(搴?, 姝ｅ€煎彸杞? 璐熷€煎乏杞?
 */
void Turn_InPlace(float angle)
{
    // 瑙掑害杞姬搴?
	float angle_rad = angle * 3.14159265f / 180.0f;
	
    // 璁＄畻姣忎釜杞瓙闇€瑕佽蛋鐨勫姬闀?cm)
	float arc_length = angle_rad * (WHEEL_BASE / 2.0f);
	
    // 姝ｈ搴?鍙宠浆): 宸﹁疆鍓嶈繘, 鍙宠疆鍚庨€€
    // 璐熻搴?宸﹁浆): 宸﹁疆鍚庨€€, 鍙宠疆鍓嶈繘
	TargetPositionL = PositionL + arc_length;
	TargetPositionR = PositionR - arc_length;
}

/**
 * 闄€铻轰华Z杞撮浂鍋忔牎鍑嗗嚱鏁?
 * 涓婄數鏃朵繚鎸侀潤姝? 閲囨牱200娆℃眰骞冲潎鍊? 绾?绉?
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
 * 鍋忚埅瑙掓洿鏂板嚱鏁?
 * 閫氳繃Z杞撮檧铻轰华瑙掗€熷害绉垎璁＄畻鍋忚埅瑙?
 * 鍙傛暟 dt: 閲囨牱鍛ㄦ湡(绉?
 */
void Yaw_Update(float dt)
{
	// 璁＄畻瑙掗€熷害(掳/s)
	float gyro_z_dps = ((float)GZ - GyroZ_Offset) / GYRO_SENSITIVITY;
	
	// 姝诲尯澶勭悊:灏忚閫熷害瑙嗕负0,鍑忓皯闈欐婕傜Щ
	if (gyro_z_dps > -GYRO_Z_DEADZONE && gyro_z_dps < GYRO_Z_DEADZONE)
	{
		gyro_z_dps = 0.0f;
	}
	GyroZ_Dps = gyro_z_dps;
	
    // 绉垎寰楀埌鍋忚埅瑙?
	YawAngle += gyro_z_dps * dt;
	
    // 闄愬埗鍦?-180掳~180掳 鑼冨洿鍐?
	if (YawAngle > 180.0f) YawAngle -= 360.0f;
	if (YawAngle < -180.0f) YawAngle += 360.0f;
}

/**
 * 闃熷垪寮忔粦鍔ㄥ钩鍧囨护娉?瀹屽杽闃熷垪鏈弧閫昏緫)
 * 鍔熻兘: 1. 闃熷垪鏈弧鍙叆闃? 鎸夋湁鏁堜釜鏁扮畻鍧囧€? 2. 闃熷垪宸叉弧鍏ラ槦鍑洪槦, 鎸夐槦鍒楅暱搴︾畻鍧囧€?
 */
void Speed_Queue_Filter(float new_L, float new_R)
{
    // ===== 绗竴姝? 澶勭悊闃熷垪鏈弧鐨勬儏鍐?=====
    if (Queue_Count < FILTER_QUEUE_LEN)
    {
        // 闃熷垪鏈弧锛氱洿鎺ュ叆闃燂紙涓嶅垹闄ゆ棫鏁版嵁锛夛紝鏈夋晥璁℃暟+1
        SpeedL_Queue[Queue_Index] = new_L;
        SpeedR_Queue[Queue_Index] = new_R;
        
        // 绱姞鏂版暟鎹埌鎬诲拰
        SpeedL_Sum += new_L;
        SpeedR_Sum += new_R;
        
        // 鏈夋晥鏁版嵁涓暟+1
        Queue_Count++;
        
        // 璁＄畻鍧囧€? 鎬诲拰 / 鏈夋晥鏁版嵁涓暟, 閬垮厤鏁板€肩█閲?
        SpeedL_Filtered = SpeedL_Sum / Queue_Count;
        SpeedR_Filtered = SpeedR_Sum / Queue_Count;
    }
    // ===== 绗簩姝? 闃熷垪宸叉弧鐨勬儏鍐?鍘烣IFO閫昏緫) =====
    else
    {
        // 1. 鍒犻櫎鏈€鏃╃殑鏁版嵁锛堝噺鍘诲嵆灏嗚瑕嗙洊鐨勬棫鍊硷級
        SpeedL_Sum -= SpeedL_Queue[Queue_Index];
        SpeedR_Sum -= SpeedR_Queue[Queue_Index];
        
        // 2. 鏂版暟鎹叆闃?瑕嗙洊鏈€鏃╃殑鏁版嵁)
        SpeedL_Queue[Queue_Index] = new_L;
        SpeedR_Queue[Queue_Index] = new_R;
        
        // 3. 绱姞鏂版暟鎹埌鎬诲拰
        SpeedL_Sum += new_L;
        SpeedR_Sum += new_R;
        
        // 4. 璁＄畻鍧囧€? 鎬诲拰 / 闃熷垪闀垮害(婊￠槦鍒?
        SpeedL_Filtered = SpeedL_Sum / FILTER_QUEUE_LEN;
        SpeedR_Filtered = SpeedR_Sum / FILTER_QUEUE_LEN;
    }
    
    // ===== 绗笁姝ワ細鏇存柊鍐欏叆绱㈠紩锛堢幆褰㈢Щ鍔級 =====
    Queue_Index++;
    if (Queue_Index >= FILTER_QUEUE_LEN)
    {
        Queue_Index = 0;
    }
}

#if SENSOR_TEST_ENABLE
static void SensorTest_Printf(const char *format, ...)
{
    char buffer[180];
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

static void SensorTest_PrintSignedCenti(const char *name, int16_t value)
{
    int16_t frac = value % 100;
    if (frac < 0)
    {
        frac = -frac;
    }
    SensorTest_Printf("%s:%d.%02d ", name, value / 100, frac);
}

int main(void)
{
    float temp = 0.0f;
    float humi = 0.0f;
    uint8_t sht_ok;
    uint8_t sht_err;
    uint8_t mpu_id;

    Serial1_Init();
    Delay_ms(300);

    SensorTest_Printf("\r\n==== SENSOR TEST: SHT31 + MPU6050 ====\r\n");
    SensorTest_Printf("USART1 baud: 921600\r\n");

    MPU6050_Init();
    Delay_ms(100);
    mpu_id = MPU6050_GetID();
    SensorTest_Printf("MPU6050 ID:0x%02X\r\n", mpu_id);

    SHT31_Init();
    Delay_ms(100);

    SensorTest_Printf("Keep still, gyro calibrating...\r\n");
    GyroZ_Calibrate();
    SensorTest_Printf("GyroZ offset: ");
    SensorTest_PrintSignedCenti("", Float_To_Centi(GyroZ_Offset));
    SensorTest_Printf("\r\n");

    while (1)
    {
        MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
        Yaw_Update(0.5f);

        sht_ok = (SHT31_ReadData(&temp, &humi) == 0);
        sht_err = SHT31_GetLastError();

        SensorTest_Printf("MPU AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d ", AX, AY, AZ, GX, GY, GZ);
        SensorTest_PrintSignedCenti("Yaw", Float_To_Centi(YawAngle));
        SensorTest_Printf("SHT:%s Err:%u ", sht_ok ? "OK" : "FAIL", sht_err);
        SensorTest_PrintSignedCenti("T", Float_To_Centi(temp));
        SensorTest_PrintSignedCenti("H", Float_To_Centi(humi));
        SensorTest_Printf("\r\n");

        Delay_ms(500);
    }
}
#else
int main(void)

{
    // 妯″潡鍒濆鍖?
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
#if DEBUG_TEXT_ENABLE
    Serial1_Printf("MPU6050 ID:0x%02X\r\n", MPU6050_GetID());
#endif
#if MLX90642_ENABLE
    MLX90642_Ready = (MLX90642_App_Init() == 0);
#endif
	SHT31_Init();
    Delay_ms(500);
    // 闄€铻轰华Z杞撮浂鍋忔牎鍑? 涓婄數淇濇寔闈欐绾?绉?
	GyroZ_Calibrate();
#if DEBUG_TEXT_ENABLE
    Serial1_Printf("GyroZ offset:%.2f\r\n", GyroZ_Offset);
#endif
    Delay_ms(500);
    // 閫熷害鐜疨ID鍒濆鍖?鍐呯幆, 澧為噺寮? PWM闄愬箙0-1000)
    fp32 param_SpeedL[3] = {PID_Param_SpeedL.Kp, PID_Param_SpeedL.Ki, PID_Param_SpeedL.Kd};
    fp32 param_SpeedR[3] = {PID_Param_SpeedR.Kp, PID_Param_SpeedR.Ki, PID_Param_SpeedR.Kd};
    PID_init(&PID_SpeedL, PID_DELTA, param_SpeedL, 1000.0f, 100.0f);
    PID_init(&PID_SpeedR, PID_DELTA, param_SpeedR, 1000.0f, 100.0f);
    
    // 浣嶇疆鐜疨ID鍒濆鍖?澶栫幆, 浣嶇疆寮? 閫熷害闄愬箙卤30cm/s)
    fp32 param_PositionL[3] = {PID_Param_PositionL.Kp, PID_Param_PositionL.Ki, PID_Param_PositionL.Kd};
    fp32 param_PositionR[3] = {PID_Param_PositionR.Kp, PID_Param_PositionR.Ki, PID_Param_PositionR.Kd};
    PID_init(&PID_PositionL, PID_POSITION, param_PositionL, 30.0f, 10.0f);
    PID_init(&PID_PositionR, PID_POSITION, param_PositionR, 30.0f, 10.0f);
    
    // 瑙掑害鐜疨ID鍒濆鍖?浣嶇疆寮? 杈撳嚭宸€熼€熷害, 闄愬箙卤40cm/s)
    fp32 param_YAW[3] = {PID_Param_YAW.Kp, PID_Param_YAW.Ki, PID_Param_YAW.Kd};
    PID_init(&PID_YAW, PID_POSITION, param_YAW, 30.0f, 10.0f);
#if !MOTOR_ENCODER_TEST && !SPEED_LOOP_TEST && !TRACK_MODULE_TEST
	Timer_Init();
#endif
	
	
	Delay_ms(500);
	
	while (1)
	{
        Protocol_Poll();

        if (T_10ms == 1)
        {
            // 涓插彛1鎵撳嵃璋冭瘯鏁版嵁(瀹為檯瑙掑害, 鐩爣瑙掑害, 瑙掑害鐜疨ID杈撳叆鍊? KP, KI, KD)
            
            
            Track_Service_10ms();
            T_10ms = 0;
        }
		
        // 鍚戜笂浣嶆満鍙戦€佹暟鎹? 姣?00ms涓€娆?
		if (SendDataFlag == 1)
		{
			SendDataFlag = 0;
            MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
            SHT31_Ready = (SHT31_ReadData(&SHT31_Temperature, &SHT31_Humidity) == 0);
#if MLX90642_ENABLE
            if (MLX90642_Ready && MLX90642_App_ReadFrame(MLX90642_TempMap) == 0)
            {
            }
#endif
			
            // 鍙戦€佹暟鎹牸寮? 瑙掑害, 娓╁害, 婀垮害, 褰撳墠閫熷害
			AvgSpeed = (SpeedL_Filtered + SpeedR_Filtered) / 2.0f;
			AvgDistance = (PositionL + PositionR) / 2.0f;
			
            Battery_Update();
            Telemetry_SendBasic();
		}
		
		// 澶勭悊USART1鎺ユ敹鍒扮殑鍛戒护
		if (Serial1_RxFlag == 1)
		{
			// 瑙ｆ瀽鍛戒护鏍煎紡: "MOVE,璺濈"
			if (strncmp(Serial1_RxPacket, "MOVE,", 5) == 0)
			{
				float distance = 0;
				sscanf(Serial1_RxPacket + 5, "%f", &distance);
				
                // 閫€鍑鸿浆鍚戞ā寮? 杩涘叆鐩寸嚎妯″紡
				TurnMode = 0;
				// 璁剧疆鐩爣浣嶇疆(涓よ疆鍚屾椂鍓嶈繘鐩稿悓璺濈)
				TargetPositionL = PositionL + distance;
				TargetPositionR = PositionR + distance;
			}
            // 瑙ｆ瀽鍛戒护鏍煎紡: "TURN,瑙掑害" (姝ｅ€煎彸杞? 璐熷€煎乏杞?
			else if (strncmp(Serial1_RxPacket, "TURN,", 5) == 0)
			{
				float angle_offset = 0;
				sscanf(Serial1_RxPacket + 5, "%f", &angle_offset);
                // 鐩爣鍋忚埅瑙?= 褰撳墠鍋忚埅瑙?+ 杈撳叆瑙掑害
				turn_angle = YawAngle + angle_offset;
				// 杩涘叆杞悜妯″紡
				TurnMode = 1;
                // 娓呴浂瑙掑害鐜疨ID绱姞鍊? 閬垮厤涓婃娈嬬暀褰卞搷
				PID_clear(&PID_YAW);
				PID_clear(&PID_SpeedL);
				PID_clear(&PID_SpeedR);
			}
			else if (strcmp(Serial1_RxPacket, "STOP") == 0)
			{
                // 鍋滄鍛戒护: 閫€鍑鸿浆鍚戞ā寮? 鐩爣浣嶇疆璁句负褰撳墠浣嶇疆
				TurnMode = 0;
				TargetPositionL = PositionL;
				TargetPositionR = PositionR;
			}
			else if (strcmp(Serial1_RxPacket, "RESET") == 0)
			{
                // 浣嶇疆娓呴浂, 閫€鍑鸿浆鍚戞ā寮?
				TurnMode = 0;
				Position_Reset();
			}
			
			Serial1_RxFlag = 0;  // 娓呴櫎鎺ユ敹鏍囧織
		}
		

        // 璇诲彇MPU6050鏁版嵁(涓诲惊鐜珮棰戞洿鏂? 淇濊瘉涓柇浣跨敤鏈€鏂癎Z鍊?
		
        // 绗?琛屾樉绀哄钩鍧囪椹惰窛绂?cm)
        // 绗?琛屾樉绀虹洰鏍囦綅缃?cm)
 
        // 绗?琛屾樉绀哄疄闄呴€熷害(cm/s)
 
        // 绗?琛屽乏渚ф樉绀虹洰鏍囬€熷害(cm/s); 鍙充晶鍋忚埅瑙掑湪T_10ms涓洿鏂?
	}
}

/**
 * TIM1涓柇: 10ms涓€娆? 鍙岄棴鐜疨ID鎺у埗
 * 鎺у埗娴佺▼: 浣嶇疆绱姞 -> 浣嶇疆鐜疨ID -> 鐩爣閫熷害 -> 閫熷害婊ゆ尝 -> 閫熷害鐜疨ID -> PWM杈撳嚭
 */
#endif
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		// ========== 绗?姝? 璇诲彇缂栫爜鍣ㄥ師濮嬭剦鍐? 璁＄畻閫熷害 ==========
		SpeedL = -Encoder_Soft_Get();		
		SpeedR = Encoder_TIM3_Get();		
		SpeedL_cms = Pulse_To_Speed(SpeedL);
		SpeedR_cms = Pulse_To_Speed(SpeedR);
		
        // ========== 绗?姝? 绱姞浣嶇疆(閫熷害脳鏃堕棿, 10ms=0.01s) ==========
		PositionL += SpeedL_cms * 0.01f;
        PositionR += SpeedR_cms * 0.01f;
		
        // ========== 绗?.5姝? 鏇存柊鍋忚埅瑙? 鍦ㄦ帶鍒惰绠椾箣鍓嶇‘淇濅娇鐢ㄦ渶鏂拌搴?==========
		Yaw_Update(0.01f);
		
        // ========== 绗?姝? 鏍规嵁妯″紡閫夋嫨鎺у埗绛栫暐 ==========
		if (TurnMode == 1)
		{
            // ===== 杞悜妯″紡: 瑙掑害鐜疨ID -> 宸€熼€熷害 -> 閫熷害鐜?=====
			PID_set_param_s(&PID_YAW, &PID_Param_YAW);
			
			// 璁＄畻瑙掑害璇樊
			float YAW_Error = turn_angle - YawAngle;
			
            // 瑙掑害姝诲尯鍒ゆ柇: 璇樊灏忎簬姝诲尯鏃跺仠姝㈣浆鍚?
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
				// 瑙掑害鐜疨ID杈撳嚭宸€熼€熷害(cm/s)
				Target_YAW = PID_calc(&PID_YAW, YawAngle, turn_angle);
                // 鏈€灏忛€熷害閽充綅: 淇濊瘉鐢垫満鏈夎冻澶熸壄鐭╄浆鍔?
				if (Target_YAW > 0 && Target_YAW < YAW_MIN_SPEED) Target_YAW = YAW_MIN_SPEED;
				else if (Target_YAW < 0 && Target_YAW > -YAW_MIN_SPEED) Target_YAW = -YAW_MIN_SPEED;
                // 宸﹁疆姝ｈ浆, 鍙宠疆鍙嶈浆 -> 鍘熷湴绛夋瘮宸€熻浆鍚?
				TargetSpeedL = -Target_YAW;
				TargetSpeedR = +Target_YAW;
			}
		}
		else
		{
            // ===== 鐩寸嚎妯″紡: 浣跨敤浣嶇疆鐜帶鍒剁洰鏍囦綅缃?=====
			TargetSpeedL = 0;
			TargetSpeedR = 0;
			
            /* === 浣嶇疆鐜帶鍒朵唬鐮?=== */
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

#if TRACK_AUTO_ENABLE
            if (Track_Ready && (TargetSpeedL != 0.0f || TargetSpeedR != 0.0f))
            {
                float baseSpeed = (TargetSpeedL + TargetSpeedR) * 0.5f;

                if (baseSpeed > TRACK_BASE_SPEED_CMS) baseSpeed = TRACK_BASE_SPEED_CMS;
                else if (baseSpeed < -TRACK_BASE_SPEED_CMS) baseSpeed = -TRACK_BASE_SPEED_CMS;

                Track_TurnSpeed = (float)Track_Error * TRACK_TURN_KP;
                if (Track_TurnSpeed > TRACK_TURN_MAX_CMS) Track_TurnSpeed = TRACK_TURN_MAX_CMS;
                else if (Track_TurnSpeed < -TRACK_TURN_MAX_CMS) Track_TurnSpeed = -TRACK_TURN_MAX_CMS;

                TargetSpeedL = baseSpeed + Track_TurnSpeed;
                TargetSpeedR = baseSpeed - Track_TurnSpeed;
            }
            else
            {
                Track_TurnSpeed = 0.0f;
            }
#endif

		}
		
        // ========== 绗?姝? 閫熷害闃熷垪寮忔护娉?==========
		Speed_Queue_Filter(SpeedL_cms, SpeedR_cms);
		
        // ========== 绗?姝? 閫熷害鐜疨ID璁＄畻(鍐呯幆, 杈撳嚭PWM) ==========
        PID_set_param_s(&PID_SpeedL, &PID_Param_SpeedL);
		PID_set_param_s(&PID_SpeedR, &PID_Param_SpeedR);
        
		OutputL = PID_calc(&PID_SpeedL, SpeedL_Filtered, TargetSpeedL);
		OutputR = PID_calc(&PID_SpeedR, SpeedR_Filtered, TargetSpeedR);
		
        // ========== 绗?姝? 鐢垫満PWM杈撳嚭(闄愬箙50-1000, 閬垮厤浣庨€熸壄鐭╀笉瓒? ==========
        // 瀵筆WM杈撳嚭杩涜鏈€灏忓€奸檺骞?
		int16_t pwmL = (int16_t)OutputL;
		int16_t pwmR = (int16_t)OutputR;
		
        // 濡傛灉PWM缁濆鍊煎皬浜庢鍖? 鍒欒涓烘鍖哄€? 鍚﹀垯淇濇寔璁＄畻鍊?
		if (pwmL > 0 && pwmL < PWM_DEADZONE) pwmL = PWM_DEADZONE;
		else if (pwmL < 0 && pwmL > -PWM_DEADZONE) pwmL = -PWM_DEADZONE;
		
		if (pwmR > 0 && pwmR < PWM_DEADZONE) pwmR = PWM_DEADZONE;
		else if (pwmR < 0 && pwmR > -PWM_DEADZONE) pwmR = -PWM_DEADZONE;
		
		MotorL_SetSpeed(pwmL);
		MotorR_SetSpeed(pwmR);
		
        T_10ms = 1;
		
        // ========== 绗?姝? 鏁版嵁鍙戦€佽鏁?姣?00ms璁剧疆涓€娆℃爣蹇椾綅) ==========
		SendCounter++;
		if (SendCounter >= SEND_INTERVAL)
		{
			SendCounter = 0;
            SendDataFlag = 1;  // 璁剧疆鍙戦€佹爣蹇椾綅, 鍦ㄤ富寰幆涓彂閫?
		}
		
		// 娓呴櫎涓柇鏍囧織
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

