#include "Pid.h"

extern PID_T PID_Current;

// ====================== 初始化 ======================
void pid_init(PID_T *pid, float kp, float ki, float kd, float target, float limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = target;
    pid->limit = limit;
    
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last2_error = 0.0f;
    pid->out = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
}

// ====================== 设置目标值（关键修复） ======================
void pid_set_target(PID_T *pid, float target)
{
    pid->target = target;
    // 不清历史状态，防止输出突跳
}

// ====================== 设置参数 ======================
void pid_set_params(PID_T *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_limit(PID_T *pid, float limit)
{
    pid->limit = limit;
}

// ====================== 重置 ======================
void pid_reset(PID_T *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last2_error = 0.0f;
    pid->out = 0.0f;
    pid->p_out = 0.0f;
    pid->i_out = 0.0f;
    pid->d_out = 0.0f;
}

// ====================== 位置式PID（推荐用于电压控制） ======================
float pid_calculate_positional(PID_T *pid, float current)
{
    pid->current = current;                    // ① 记住当前实际电压
    pid->error = pid->target - current;        // ② 计算误差（最重要！）
	
	if(pid == &PID_Current) 
	{		
		pid->integral += pid->error;		
	}	
	else 
	{
    // 积分分离（可配置，这里建议误差大于目标的20%时分离）
    float sep_threshold = pid->target * 0.35f;   // ③ 计算积分分离阈值（比如目标8.2V时，阈值≈1.64V）
    if (fabsf(pid->error) > sep_threshold) 			 // ④ 如果误差很大（车偏离太远）
		{    
        pid->integral = 0.0f;                    //    就先不积累积分（避免一开始就猛踩油门导致超调）
    } 
		else 
		{                                    				 // ⑤ 如果误差不大（车已经比较接近了）
			pid->integral += pid->error;               //    慢慢积累误差（帮助彻底消除最后那点小偏差）
		}
	}
		// 积分限幅（和输出limit关联，更合理）
		float i_limit = pid->limit / (pid->ki + 0.001f);  // ⑥ 防止积分累太多（除0保护）
		if (pid->integral > i_limit)  pid->integral = i_limit;
		if (pid->integral < -i_limit) pid->integral = -i_limit;
    
	
    pid->p_out = pid->kp * pid->error;                    // ⑦ 比例项 P：误差越大，反应越猛（马上纠正方向）
    pid->i_out = pid->ki * pid->integral;                 // ⑧ 积分项 I：把过去的误差加起来，彻底消灭稳态误差
    pid->d_out = pid->kd * (pid->error - pid->last_error);// ⑨ 微分项 D：看误差变化的速度（提前“刹车”，防止晃动）

    pid->out = pid->p_out + pid->i_out + pid->d_out;      // ⑩ 把P+I+D三股力量加在一起，得到最终“油门大小”

    // 输出限幅
    if (pid->out > pid->limit)   pid->out = pid->limit;   //  不允许油门踩得太大（保护硬件）
    if (pid->out < -pid->limit)  pid->out = -pid->limit;

    pid->last_error = pid->error;                         //  记住这次误差，留给下一次算D项用
    return pid->out;                                      //  返回最终输出值，供你去调整SPWM
}

// ====================== 增量式PID（保留，供你选择） ======================
float pid_calculate_incremental(PID_T *pid, float current)
{
    pid->current = current;
    pid->error = pid->target - current;

    pid->p_out = pid->kp * (pid->error - pid->last_error);
    pid->i_out = pid->ki * pid->error;
    pid->d_out = pid->kd * (pid->error - 2 * pid->last_error + pid->last2_error);

    pid->out += pid->p_out + pid->i_out + pid->d_out;

    // 输出限幅
    if (pid->out > pid->limit)   pid->out = pid->limit;
    if (pid->out < -pid->limit)  pid->out = -pid->limit;

    pid->last2_error = pid->last_error;
    pid->last_error = pid->error;
    return pid->out;
}

// ====================== 工具函数 ======================
float pid_constrain(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void f32_PI_Init(PI_TypeDef *PI_Pamer ,float Ts, float Kp, float Ki, int16_t TH, int16_t TL)
{
    PI_Pamer->Ts = Ts;
    PI_Pamer->Kp = Kp;
    PI_Pamer->Ki = Ki;
    PI_Pamer->filter_B0 = Ts*Ki/2 + Kp;
    PI_Pamer->filter_B1 = Ts*Ki/2 - Kp;
    PI_Pamer->TH = TH;
    PI_Pamer->TL = TL;
}

float f32_PI_Calculate(PI_TypeDef *PI_Pamer ,float REF, float Sample)
{
    
    PI_Pamer->x1 = PI_Pamer->x0;
    PI_Pamer->x0 = (REF - Sample);
    
    float tmp_co;
    //tmp_co = PI_Pamer->y1 + PI_Pamer->Kp*(PI_Pamer->x0-PI_Pamer->x1) + PI_Pamer->Ts*PI_Pamer->Ki*(PI_Pamer->x0+PI_Pamer->x1)/2;
    tmp_co = PI_Pamer->y1 + PI_Pamer->filter_B0*PI_Pamer->x0 + PI_Pamer->filter_B1*PI_Pamer->x1;
    
    if(tmp_co >= PI_Pamer->TH)
        PI_Pamer->y0 = PI_Pamer->TH;
    else if(tmp_co <= PI_Pamer->TL)
        PI_Pamer->y0 = PI_Pamer->TL;
    else
        PI_Pamer->y0 = tmp_co;
    
    PI_Pamer->y1 = PI_Pamer->y0;
    return PI_Pamer->y0;
}

