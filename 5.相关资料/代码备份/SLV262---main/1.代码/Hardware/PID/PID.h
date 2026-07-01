// PID.h 中添加
#ifndef PID_H
#define PID_H

#include "stm32f10x.h"

#ifndef NULL
#define NULL ((void *)0)
#endif

// 1. 适配浮点型别名（和你的工程兼容）
typedef double fp32;

// 2. PID模式宏定义（位置式/增量式）
#define PID_POSITION 0  // 位置式PID（电机速度控制推荐）
#define PID_DELTA    1  // 增量式PID

// PID参数结构体(方便debug调参)
typedef struct
{
    fp32 Kp;  // 比例系数
    fp32 Ki;  // 积分系数
    fp32 Kd;  // 微分系数
} PID_Param_t;

// 3. PID结构体定义（替换你原来的PID_TypeDef）
typedef struct
{
    uint8_t mode;          // PID模式（位置式/增量式）
    fp32 Kp, Ki, Kd;       // 比例、积分、微分系数
    fp32 max_out;          // PID总输出最大值（对应PWM范围，比如-80~80）
    fp32 max_iout;         // 积分项输出最大值（防止积分饱和）
    fp32 error[3];         // 误差缓存：[0]当前 [1]上次 [2]上上次
    fp32 Dbuf[3];          // 微分项缓存
    fp32 Pout, Iout, Dout; // 分项输出
    fp32 out;              // 总输出
    fp32 fdb, set;         // 反馈值、设定值
} pid_type_def;  // 注意：把你工程里的PID_TypeDef改成这个！

// 4. 函数声明（和你之前的PID代码对应）
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
void PID_clear(pid_type_def *pid);
void PID_set_param(pid_type_def *pid, const fp32 PID[3]);
void PID_set_param_s(pid_type_def *pid, const PID_Param_t *param);

#endif

