#ifndef __PR_H
#define __PR_H

#include "mydefine.h"

typedef struct {
    float Kp;          // 比例系数
    float Kr;          // 谐振系数
    float OMEGA_0;     // 谐振角频率
    float OMEGA_C;     // 带宽（准PR用）
    float Ts;          // 采样周期
	
		float b0;
		float b1;
		float b2;
		float a1;
		float a2;
    
    /* 内部状态变量（离散化后需要记住上两拍的值） */
    float e_k;         // 当前误差 e[k]
    float e_k1;        // 上一拍误差 e[k-1]
    float e_k2;        // 上上拍误差 e[k-2]
    float u_r_k1;      // 谐振部分上一拍输出
    float u_r_k2;      // 谐振部分上上拍输出
	
		float u_proportional;
		float	u_resonant;
		float	u_total;
    
    float output_max;  // 输出限幅最大值
    float output_min;  // 输出限幅最小值
} PR_T;

void pr_init(PR_T *pr, float kp, float kr, float omega_0, float omega_c,float ts);
float PR_Calculate(PR_T  *pr, float error);
void PR_Precompute(PR_T *pr);

#endif
