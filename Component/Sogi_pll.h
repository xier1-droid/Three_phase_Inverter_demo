#ifndef __SOGI_PLL_H
#define __SOGI_PLL_H

#include "mydefine.h"

/* SOGI 正交信号发生器（双线性变换系数法，结构仿照 PR_T） */
typedef struct {
    float k;           // 阻尼系数
    float omega0;      // 谐振中心角频率（额定电网角频率）
    float Ts;          // 采样周期

    /* 双线性变换预计算系数 */
    float d_b0, d_b1, d_b2;   // 同相通道(v_alpha)分子系数
    float q_b0, q_b1, q_b2;   // 正交通道(v_beta)分子系数
    float a1, a2;             // 公共分母系数

    /* 内部状态（直接型II，两通道共用一套状态） */
    float w1, w2;      // w[n-1], w[n-2]
} SOGI_T;

/* PLL专用环路滤波器（双线性PI，角频率量纲，结构仿照 PI_TypeDef） */
typedef struct {
    float Kp, Ki, Ts;
    float b0, b1;          // 双线性PI系数
    float e_prev;          // e[n-1]
    float out_prev;        // Δω[n-1]
    float out_min, out_max;
} PLL_LoopFilter_T;

/* 顶层 SOGI-PLL */
typedef struct {
    SOGI_T sogi;
    PLL_LoopFilter_T loopfilter;

    float omega0;      // 额定角频率前馈
    float Ts;

    float v_alpha, v_beta;  // SOGI输出（同相/正交分量）
    float v_q;              // Park变换鉴相误差

    float theta;       // 当前估计相位 [0, 2*pi)
    float freq_hz;      // 估计频率
} SOGI_PLL_T;

/**
 * @brief 初始化SOGI正交信号发生器并预计算离散化系数
 * @param s SOGI实例
 * @param k 阻尼系数
 * @param omega0 谐振中心角频率(rad/s)
 * @param ts 采样周期(s)
 */
void sogi_init(SOGI_T *s, float k, float omega0, float ts);

/**
 * @brief 预计算SOGI双线性变换离散化系数，仅在sogi_init时调用一次
 * @param s SOGI实例（需先设置好 k, omega0, Ts）
 */
void SOGI_Precompute(SOGI_T *s);

/**
 * @brief SOGI每拍递推计算
 * @param s SOGI实例
 * @param v_in 输入采样(电网电压瞬时值)
 * @param v_alpha 输出：同相分量
 * @param v_beta 输出：正交分量(滞后90°)
 */
void SOGI_Calculate(SOGI_T *s, float v_in, float *v_alpha, float *v_beta);

/**
 * @brief 初始化PLL环路滤波器并预计算离散化系数
 * @param lf 环路滤波器实例
 * @param kp 比例系数
 * @param ki 积分系数
 * @param ts 采样周期(s)
 * @param out_min 输出下限(Δω, rad/s)
 * @param out_max 输出上限(Δω, rad/s)
 */
void pll_loopfilter_init(PLL_LoopFilter_T *lf, float kp, float ki, float ts, float out_min, float out_max);

/**
 * @brief 运行时修改PLL环路滤波器的Kp/Ki并重新计算双线性系数，便于串口在线调参
 * @param lf 环路滤波器实例
 * @param kp 新的比例系数
 * @param ki 新的积分系数
 */
void pll_loopfilter_set_kpki(PLL_LoopFilter_T *lf, float kp, float ki);

/**
 * @brief PLL环路滤波器每拍递推计算（带输出限幅）
 * @param lf 环路滤波器实例
 * @param e 鉴相误差(v_q)
 * @return Δω(rad/s)
 */
float PLL_LoopFilter_Calculate(PLL_LoopFilter_T *lf, float e);

/**
 * @brief 初始化SOGI-PLL
 * @param s SOGI-PLL实例
 * @param k SOGI阻尼系数
 * @param freq_nominal 额定电网频率(Hz)，如50.0f
 * @param ts 采样周期(s)，对应控制环采样率
 * @param pi_kp PLL环路滤波器比例系数
 * @param pi_ki PLL环路滤波器积分系数
 * @param dw_limit Δω限幅幅值(rad/s)，限幅范围为[-dw_limit, dw_limit]
 */
void sogi_pll_init(SOGI_PLL_T *s, float k, float freq_nominal, float ts, float pi_kp, float pi_ki, float dw_limit);

/**
 * @brief SOGI-PLL每拍更新：SOGI正交信号 -> Park变换鉴相 -> 环路滤波 -> VCO相位积分
 * @param s SOGI-PLL实例
 * @param v_in 输入采样(电网电压瞬时值，物理量纲)
 */
void sogi_pll_update(SOGI_PLL_T *s, float v_in);

#endif
