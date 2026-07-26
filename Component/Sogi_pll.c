#include "Sogi_pll.h"

#define PLL_TWO_PI 6.28318530718f

void sogi_init(SOGI_T *s, float k, float omega0, float ts)
{
    s->k = k;
    s->omega0 = omega0;
    s->Ts = ts;

    s->w1 = 0.0f;
    s->w2 = 0.0f;

    SOGI_Precompute(s);
}

/*
 * 对D(s)、Q(s)做双线性变换 s = (2/Ts)(z-1)/(z+1)，两通道共用同一分母。
 * 令 c = 2/Ts，A = c^2 + k*w0*c + w0^2（归一化因子）：
 *   a1 = 2*(w0^2 - c^2) / A
 *   a2 = (c^2 - k*w0*c + w0^2) / A
 *   d_b0 =  k*w0*c / A,  d_b1 = 0,          d_b2 = -k*w0*c / A
 *   q_b0 =  k*w0^2 / A,  q_b1 = 2*k*w0^2/A, q_b2 =  k*w0^2 / A
 */
void SOGI_Precompute(SOGI_T *s)
{
    float k = s->k;
    float w0 = s->omega0;
    float c = 2.0f / s->Ts;

    float A = c * c + k * w0 * c + w0 * w0;

    s->a1 = (2.0f * (w0 * w0 - c * c)) / A;
    s->a2 = (c * c - k * w0 * c + w0 * w0) / A;

    s->d_b0 = (k * w0 * c) / A;
    s->d_b1 = 0.0f;
    s->d_b2 = -(k * w0 * c) / A;

    s->q_b0 = (k * w0 * w0) / A;
    s->q_b1 = (2.0f * k * w0 * w0) / A;
    s->q_b2 = (k * w0 * w0) / A;
}

void SOGI_Calculate(SOGI_T *s, float v_in, float *v_alpha, float *v_beta)
{
    float w = v_in - s->a1 * s->w1 - s->a2 * s->w2;

    *v_alpha = s->d_b0 * w + s->d_b1 * s->w1 + s->d_b2 * s->w2;
    *v_beta  = s->q_b0 * w + s->q_b1 * s->w1 + s->q_b2 * s->w2;

    s->w2 = s->w1;
    s->w1 = w;
}

void pll_loopfilter_init(PLL_LoopFilter_T *lf, float kp, float ki, float ts, float out_min, float out_max)
{
    lf->Kp = kp;
    lf->Ki = ki;
    lf->Ts = ts;

    lf->b0 = kp + ki * ts * 0.5f;
    lf->b1 = ki * ts * 0.5f - kp;

    lf->e_prev = 0.0f;
    lf->out_prev = 0.0f;

    lf->out_min = out_min;
    lf->out_max = out_max;
}

float PLL_LoopFilter_Calculate(PLL_LoopFilter_T *lf, float e)
{
    float out = lf->out_prev + lf->b0 * e + lf->b1 * lf->e_prev;

    if (out > lf->out_max) out = lf->out_max;
    if (out < lf->out_min) out = lf->out_min;

    lf->e_prev = e;
    lf->out_prev = out;

    return out;
}

void pll_loopfilter_set_kpki(PLL_LoopFilter_T *lf, float kp, float ki)
{
    lf->Kp = kp;
    lf->Ki = ki;

    lf->b0 = kp + ki * lf->Ts * 0.5f;
    lf->b1 = ki * lf->Ts * 0.5f - kp;

    /* 避免切换参数瞬间历史误差项造成输出突跳 */
    lf->e_prev = 0.0f;
    lf->out_prev = 0.0f;
}

void sogi_pll_init(SOGI_PLL_T *s, float k, float freq_nominal, float ts, float pi_kp, float pi_ki, float dw_limit)
{
    s->omega0 = PLL_TWO_PI * freq_nominal;
    s->Ts = ts;

    sogi_init(&s->sogi, k, s->omega0, ts);
    pll_loopfilter_init(&s->loopfilter, pi_kp, pi_ki, ts, -dw_limit, dw_limit);

    s->v_alpha = 0.0f;
    s->v_beta = 0.0f;
    s->v_q = 0.0f;

    s->theta = 0.0f;
    s->freq_hz = freq_nominal;
}

void sogi_pll_update(SOGI_PLL_T *s, float v_in)
{
    float dw, omega;

    SOGI_Calculate(&s->sogi, v_in, &s->v_alpha, &s->v_beta);

    /* Park变换鉴相：锁定时 v_q -> 0 */
 //   s->v_q = -s->v_alpha * sinf(s->theta) + s->v_beta * cosf(s->theta);//固定相差90°，电网电压为sin
    s->v_q = s->v_alpha * cosf(s->theta) + s->v_beta * sinf(s->theta);//固定相差0°，电网电压为sin

    dw = PLL_LoopFilter_Calculate(&s->loopfilter, s->v_q);
    omega = s->omega0 + dw;

    s->theta += omega * s->Ts;
    if (s->theta >= PLL_TWO_PI) s->theta -= PLL_TWO_PI;
    if (s->theta < 0.0f) s->theta += PLL_TWO_PI;

    s->freq_hz = omega / PLL_TWO_PI;
}
