#include "Pr.h"

// 初始化PR
void pr_init(PR_T *pr, float kp, float kr, float omega_0, float omega_c,float ts) {
		pr->Kp        = kp;
    pr->Kr        = kr;
    pr->OMEGA_0   = omega_0;
    pr->OMEGA_C   = omega_c;
    pr->Ts        = ts;
    
    /* 清空状态变量 */
    pr->e_k   = 0.0f;
    pr->e_k1  = 0.0f;
    pr->e_k2  = 0.0f;
    pr->u_r_k1 = 0.0f;
    pr->u_r_k2 = 0.0f;
	
		pr->u_proportional = 0.0f;
		pr->u_resonant = 0.0f;
		pr->u_total = 0.0f;		
    
    pr->output_max =  1.0f;   // 限幅（调制比，不能超过1）
    pr->output_min = -1.0f;
}


float PR_Calculate(PR_T *pr, float error)
{
    float u_resonant;
    float u_total;

    /* 谐振部分差分方程（系数已经提前算好） */
    u_resonant =  pr->b0 * error
               +  pr->b2 * pr->e_k2          /* b1=0，省略 */
               -  pr->a1 * pr->u_r_k1
               -  pr->a2 * pr->u_r_k2;

    /* 更新状态变量 */
    pr->e_k2   = pr->e_k1;
    pr->e_k1   = error;
    pr->u_r_k2 = pr->u_r_k1;
    pr->u_r_k1 = u_resonant;

    /* 比例 + 谐振 */
    u_total = pr->Kp * error + u_resonant;

    /* 限幅 */
    if(u_total >  pr->output_max) u_total =  pr->output_max;
    if(u_total <  pr->output_min) u_total =  pr->output_min;

    return u_total;
}

/*
 * 预计算PR离散化系数（只需要算一次，放在初始化函数里）
 * 参数：Kp, Kr, wc, w0, Ts
 */
void PR_Precompute(PR_T *pr)
{
    float wc  = pr->OMEGA_C;
    float w0  = pr->OMEGA_0;
    float Ts_ = pr->Ts;
    float Kr_ = pr->Kr;

    /* 分母公因子 */
    float den = 4.0f + 4.0f*wc*Ts_ + (w0*w0 + wc*wc)*Ts_*Ts_;
    //        = 4 + 4ωcTs + (ω02+ωc2)Ts2

    /* 谐振部分分子系数（b0, b1, b2） */
    pr->b0 =  (4.0f * Kr_ * wc * Ts_) / den;
    pr->b1 =  0.0f;   /* 中间项为零（奇对称） */
    pr->b2 = -(4.0f * Kr_ * wc * Ts_) / den;

    /* 谐振部分分母系数（a1, a2，注意符号） */
    pr->a1 = (2.0f*(w0*w0 + wc*wc)*Ts_*Ts_ - 8.0f) / den;
    pr->a2 = (4.0f - 4.0f*wc*Ts_ + (w0*w0 + wc*wc)*Ts_*Ts_) / den;

    /* 打印出来验证（调试用，串口输出） */
    // printf("b0=%.6f b2=%.6f a1=%.6f a2=%.6f\n", pr->b0, pr->b2, pr->a1, pr->a2);
}
