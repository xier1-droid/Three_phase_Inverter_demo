#ifndef __PID_H
#define __PID_H

#include "mydefine.h"

/**
 * @brief PID控制器结构体（核心数据结构）
 * @note 所有PID相关变量都放在这里，方便管理和传递
 */
typedef struct
{
    float kp;           // 比例系数（P）：决定响应速度，越大越快但容易超调
    float ki;           // 积分系数（I）：消除稳态误差，越大消除越快但容易积分饱和
    float kd;           // 微分系数（D）：抑制震荡和超调，越大越“刹车”猛

    float target;       // 目标值（设定值），比如你想让电压稳定在8.2V
    float current;      // 当前实际值（反馈值），比如标定后的Voltage_Rms
    float error;        // 当前误差 = target - current

    float integral;     // 积分累计值（I项的核心）
    float last_error;   // 上一次误差（用于D项计算）
    float last2_error;  // 上上一次误差（增量式PID需要）

    float out;          // PID最终输出值（这就是你要用来调整SPWM占空比的数值）
    float p_out;        // 比例项输出（单独拆出来方便调试）
    float i_out;        // 积分项输出
    float d_out;        // 微分项输出

    float limit;        // 输出限幅值（防止输出过大损坏硬件，比如SPWM占空比不能超过4000）
} PID_T;

/* ====================== 初始化与设置函数 ====================== */

/**
 * @brief 初始化PID控制器
 * @param pid     PID结构体指针
 * @param kp      比例系数
 * @param ki      积分系数
 * @param kd      微分系数
 * @param target  初始目标值
 * @param limit   输出限幅值
 * @note 第一次使用PID前必须调用这个函数
 */
void pid_init(PID_T *pid, float kp, float ki, float kd, float target, float limit);

/**
 * @brief 设置新的目标值（推荐使用，不会清零历史状态）
 * @param pid     PID结构体指针
 * @param target  新目标值
 * @note 改目标时用这个，避免输出突跳
 */
void pid_set_target(PID_T *pid, float target);

/**
 * @brief 动态修改PID参数
 * @param pid  PID结构体指针
 * @param kp   新比例系数
 * @param ki   新积分系数
 * @param kd   新微分系数
 */
void pid_set_params(PID_T *pid, float kp, float ki, float kd);

/**
 * @brief 修改输出限幅值
 * @param pid    PID结构体指针
 * @param limit  新限幅值
 */
void pid_set_limit(PID_T *pid, float limit);

/**
 * @brief 重置PID所有内部状态（积分、清误差等）
 * @param pid  PID结构体指针
 * @note 系统出错或重新启动时使用
 */
void pid_reset(PID_T *pid);

/* ====================== PID计算函数 ====================== */

/**
 * @brief 位置式PID计算（推荐用于电压/电流闭环控制）
 * @param pid      PID结构体指针
 * @param current  当前实际反馈值（标定后的电压有效值）
 * @return         PID输出值（用于调整SPWM占空比）
 * @note 位置式对稳态误差控制更好，适合你的电压稳定需求
 */
float pid_calculate_positional(PID_T *pid, float current);

/**
 * @brief 增量式PID计算（可选使用）
 * @param pid      PID结构体指针
 * @param current  当前实际反馈值
 * @return         PID输出增量
 * @note 增量式输出更平滑，但稳态误差稍大
 */
float pid_calculate_incremental(PID_T *pid, float current);

/* ====================== 工具函数 ====================== */

/**
 * @brief 限幅函数（通用）
 * @param value 当前值
 * @param min   最小允许值
 * @param max   最大允许值
 * @return      限幅后的值
 */
float pid_constrain(float value, float min, float max);


typedef struct
{
    float x0, x1;
    float Kp, Ki ,Ts;
    float y0, y1;
    float filter_B0, filter_B1;
    int16_t TH;
    int16_t TL;
} PI_TypeDef;

void f32_PI_Init(PI_TypeDef *PI_Pamer ,float Ts, float Kp, float Ki, int16_t TH, int16_t TL);
float f32_PI_Calculate(PI_TypeDef *PI_Pamer ,float REF, float Sample);
#endif 
