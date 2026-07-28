#include "Dq_control.h"

#include <math.h>
#include <stddef.h>

#define DQ_OUTER_DIVIDER_RELOAD       3U

typedef struct
{
    float integral;
} DqPiState;

static DqPiState outer_d_pi;
static DqPiState outer_q_pi;
static DqPiState current_d_pi;
static DqPiState current_q_pi;

static volatile float outer_kp; // 外环比例增益
static volatile float outer_ki; // 外环积分增益（已包含Ts离散步长，无需外部乘周期）
static volatile float current_kp; // 内环比例增益
static volatile float current_ki; // 内环积分增益

static float held_id_ref;//缓存外环输出电流指令
static float held_iq_ref;
static float voltage_excess_d;//电压矢量饱和余量
static float voltage_excess_q;
static uint8_t outer_divider;//外环分频器，3 个电流周期执行一次电压环
static uint8_t held_current_ref_limited;//外环电流指令矢量是否饱和标志
static uint8_t previous_voltage_limited;//上一周期电压是否饱和，用于抗饱和积分冻结判定
static DqCascadeStatus latest_status;//完整控制环状态快照

static float DqVectorMagnitudeSquared(float d, float q)//计算 dq 矢量模平方
{
    return d * d + q * q;
}

static uint8_t DqLimitVector(float *d, float *q, float limit)//矢量等比例限幅
{
    float magnitude_squared;
    float limit_squared;

    if (limit <= 0.0f)
    {
        *d = 0.0f;
        *q = 0.0f;
        return 1U;
    }

    magnitude_squared = DqVectorMagnitudeSquared(*d, *q);
    limit_squared = limit * limit;
    if (magnitude_squared > limit_squared)
    {
        float scale = limit / sqrtf(magnitude_squared);
        *d *= scale;
        *q *= scale;
        return 1U;
    }

    return 0U;
}

static void DqUpdateOuterLoop(const DqCascadeInput *input)
{
    float error_d = input->vd_ref - input->vd;
    float error_q = input->vq_ref - input->vq;
    float candidate_integral_d = outer_d_pi.integral
                                 + outer_ki * outer_kp * error_d;
    float candidate_integral_q = outer_q_pi.integral
                                 + outer_ki * outer_kp * error_q;
    float candidate_d = outer_kp * error_d + candidate_integral_d;
    float candidate_q = outer_kp * error_q + candidate_integral_q;
    float old_d = outer_kp * error_d + outer_d_pi.integral;
    float old_q = outer_kp * error_q + outer_q_pi.integral;
    float limited_candidate_d = candidate_d;
    float limited_candidate_q = candidate_q;
    float limited_old_d = old_d;
    float limited_old_q = old_q;
    float candidate_magnitude_squared;
    float old_magnitude_squared;
    float integration_step_d;
    float integration_step_q;
    uint8_t candidate_limited;
    uint8_t old_limited;
    uint8_t allow_integration;

    candidate_magnitude_squared = DqVectorMagnitudeSquared(candidate_d,
                                                            candidate_q);
    old_magnitude_squared = DqVectorMagnitudeSquared(old_d, old_q);
    candidate_limited = DqLimitVector(&limited_candidate_d,
                                      &limited_candidate_q,
                                      DQ_CURRENT_REF_LIMIT_A);
    old_limited = DqLimitVector(&limited_old_d,
                                &limited_old_q,
                                DQ_CURRENT_REF_LIMIT_A);

    allow_integration = (uint8_t)((candidate_limited == 0U) ||
                        (candidate_magnitude_squared < old_magnitude_squared));

    integration_step_d = candidate_integral_d - outer_d_pi.integral;
    integration_step_q = candidate_integral_q - outer_q_pi.integral;
    if ((previous_voltage_limited != 0U) &&
        ((integration_step_d * voltage_excess_d +
          integration_step_q * voltage_excess_q) >= 0.0f))
    {
        allow_integration = 0U;
    }

    if (allow_integration != 0U)
    {
        outer_d_pi.integral = candidate_integral_d;
        outer_q_pi.integral = candidate_integral_q;
        held_id_ref = limited_candidate_d;
        held_iq_ref = limited_candidate_q;
        held_current_ref_limited = candidate_limited;
    }
    else
    {
        held_id_ref = limited_old_d;
        held_iq_ref = limited_old_q;
        held_current_ref_limited = old_limited;
    }
}

static void DqUpdateCurrentLoop(const DqCascadeInput *input,
                                DqCascadeOutput *output)
{
    float error_d = held_id_ref - input->id;
    float error_q = held_iq_ref - input->iq;
		//候选积分项（新积分）
    float candidate_integral_d = current_d_pi.integral
                                 + current_ki * current_kp * error_d;
    float candidate_integral_q = current_q_pi.integral
                                 + current_ki * current_kp * error_q;
		//候选输出（使用新积分）
    float candidate_correction_d = current_kp * error_d
                                   + candidate_integral_d;
    float candidate_correction_q = current_kp * error_q
                                   + candidate_integral_q;
		// 旧输出（沿用上次积分，饱和时备用）
    float old_correction_d = current_kp * error_d + current_d_pi.integral;
    float old_correction_q = current_kp * error_q + current_q_pi.integral;
		
    float candidate_correction_raw_squared;
    float old_correction_raw_squared;
    float candidate_ud;
    float candidate_uq;
    float old_ud;
    float old_uq;
    float limited_candidate_ud;
    float limited_candidate_uq;
    float limited_old_ud;
    float limited_old_uq;
    float candidate_voltage_raw_squared;
    float old_voltage_raw_squared;
    float decoupling_d;
    float decoupling_q;
    float vector_limit;
    uint8_t candidate_correction_limited;
    uint8_t old_correction_limited;
    uint8_t candidate_voltage_limited;
    uint8_t old_voltage_limited;
    uint8_t allow_integration;

		//候选输出、旧输出分别做电流矢量限幅
    candidate_correction_raw_squared =
        DqVectorMagnitudeSquared(candidate_correction_d,
                                 candidate_correction_q);
    old_correction_raw_squared = DqVectorMagnitudeSquared(old_correction_d,
                                                           old_correction_q);
    candidate_correction_limited =
        DqLimitVector(&candidate_correction_d,
                      &candidate_correction_q,
                      DQ_CURRENT_CORRECTION_LIMIT_V);
    old_correction_limited = DqLimitVector(&old_correction_d,
                                            &old_correction_q,
                                            DQ_CURRENT_CORRECTION_LIMIT_V);
		
		// dq 交叉耦合前馈解耦
    decoupling_d = -input->omega_rad_s * DQ_FILTER_L_H * input->iq;
    decoupling_q = input->omega_rad_s * DQ_FILTER_L_H * input->id;
		
		//输出电压合成
    candidate_ud = input->vd + candidate_correction_d + decoupling_d;
    candidate_uq = input->vq + candidate_correction_q + decoupling_q;
    old_ud = input->vd + old_correction_d + decoupling_d;
    old_uq = input->vq + old_correction_q + decoupling_q;
		
		//SVPWM 最大电压矢量限幅
    candidate_voltage_raw_squared = DqVectorMagnitudeSquared(candidate_ud,
                                                              candidate_uq);
    old_voltage_raw_squared = DqVectorMagnitudeSquared(old_ud, old_uq);

    vector_limit = DQ_VOLTAGE_UTILIZATION * input->vdc
                   * DQ_INV_SQRT_THREE;
    limited_candidate_ud = candidate_ud;
    limited_candidate_uq = candidate_uq;
    limited_old_ud = old_ud;
    limited_old_uq = old_uq;
    candidate_voltage_limited = DqLimitVector(&limited_candidate_ud,
                                               &limited_candidate_uq,
                                               vector_limit);
    old_voltage_limited = DqLimitVector(&limited_old_ud,
                                         &limited_old_uq,
                                         vector_limit);

    allow_integration = (uint8_t)(((candidate_correction_limited == 0U) ||
                         (candidate_correction_raw_squared <
                          old_correction_raw_squared)) &&
                        ((candidate_voltage_limited == 0U) ||
                         (candidate_voltage_raw_squared <
                          old_voltage_raw_squared)));

    if (allow_integration != 0U)
    {
        current_d_pi.integral = candidate_integral_d;
        current_q_pi.integral = candidate_integral_q;
        output->ud_cmd = limited_candidate_ud;
        output->uq_cmd = limited_candidate_uq;
        output->current_correction_limited = candidate_correction_limited;
        output->voltage_limited = candidate_voltage_limited;
        voltage_excess_d = candidate_ud - limited_candidate_ud;
        voltage_excess_q = candidate_uq - limited_candidate_uq;
    }
    else
    {
        output->ud_cmd = limited_old_ud;
        output->uq_cmd = limited_old_uq;
        output->current_correction_limited = old_correction_limited;
        output->voltage_limited = old_voltage_limited;
        voltage_excess_d = old_ud - limited_old_ud;
        voltage_excess_q = old_uq - limited_old_uq;
    }

    previous_voltage_limited = output->voltage_limited;
}

void DqCascade_Reset(void)
{
    outer_d_pi.integral = 0.0f;
    outer_q_pi.integral = 0.0f;
    current_d_pi.integral = 0.0f;
    current_q_pi.integral = 0.0f;
    held_id_ref = 0.0f;
    held_iq_ref = 0.0f;
    voltage_excess_d = 0.0f;
    voltage_excess_q = 0.0f;
    outer_divider = 0U;
    held_current_ref_limited = 0U;
    previous_voltage_limited = 0U;

    latest_status.vdc = 0.0f;
    latest_status.vd_ref = 0.0f;
    latest_status.vd = 0.0f;
    latest_status.vq = 0.0f;
    latest_status.id_ref = 0.0f;
    latest_status.id = 0.0f;
    latest_status.iq_ref = 0.0f;
    latest_status.iq = 0.0f;
    latest_status.ud_cmd = 0.0f;
    latest_status.uq_cmd = 0.0f;
    latest_status.outer_kp = outer_kp;
    latest_status.outer_ki = outer_ki;
    latest_status.current_kp = current_kp;
    latest_status.current_ki = current_ki;
    latest_status.current_ref_limited = 0U;
    latest_status.current_correction_limited = 0U;
    latest_status.voltage_limited = 0U;
}

void DqCascade_Step(const DqCascadeInput *input,
                    DqCascadeOutput *output)
{
    if ((input == NULL) || (output == NULL))
    {
        return;
    }

    if (outer_divider == 0U)
    {
        DqUpdateOuterLoop(input);
        outer_divider = DQ_OUTER_DIVIDER_RELOAD;
    }
    else
    {
        outer_divider--;
    }

    output->id_ref = held_id_ref;
    output->iq_ref = held_iq_ref;
    output->current_ref_limited = held_current_ref_limited;
    DqUpdateCurrentLoop(input, output);

    latest_status.vdc = input->vdc;
    latest_status.vd_ref = input->vd_ref;
    latest_status.vd = input->vd;
    latest_status.vq = input->vq;
    latest_status.id_ref = output->id_ref;
    latest_status.id = input->id;
    latest_status.iq_ref = output->iq_ref;
    latest_status.iq = input->iq;
    latest_status.ud_cmd = output->ud_cmd;
    latest_status.uq_cmd = output->uq_cmd;
    latest_status.outer_kp = outer_kp;
    latest_status.outer_ki = outer_ki;
    latest_status.current_kp = current_kp;
    latest_status.current_ki = current_ki;
    latest_status.current_ref_limited = output->current_ref_limited;
    latest_status.current_correction_limited =
        output->current_correction_limited;
    latest_status.voltage_limited = output->voltage_limited;
}

void DqCascade_SetOuterKp(float value)
{
    if ((value >= 0.0f) && (value <= DQ_OUTER_KP_MAX))
    {
        outer_kp = value;
    }
}

void DqCascade_SetOuterKi(float value)
{
    if ((value >= 0.0f) && (value <= DQ_OUTER_KI_MAX))
    {
        outer_ki = value;
    }
}

void DqCascade_SetCurrentKp(float value)
{
    if ((value >= 0.0f) && (value <= DQ_CURRENT_KP_MAX))
    {
        current_kp = value;
    }
}

void DqCascade_SetCurrentKi(float value)
{
    if ((value >= 0.0f) && (value <= DQ_CURRENT_KI_MAX))
    {
        current_ki = value;
    }
}

float DqCascade_GetOuterKp(void)
{
    return outer_kp;
}

float DqCascade_GetOuterKi(void)
{
    return outer_ki;
}

float DqCascade_GetCurrentKp(void)
{
    return current_kp;
}

float DqCascade_GetCurrentKi(void)
{
    return current_ki;
}

void DqCascade_GetStatus(DqCascadeStatus *status)
{
    if (status != NULL)
    {
        *status = latest_status;
    }
}
