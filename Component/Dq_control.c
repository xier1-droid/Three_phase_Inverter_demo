#include "Dq_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DQ_OUTER_DIVIDER_RELOAD       3U
#define DQ_PI_CORRECTION_LIMIT_V      15.0f
#define DQ_SAMPLE_HALF_STEP_COS       0.999969157f
#define DQ_SAMPLE_HALF_STEP_SIN       0.007853901f
#define DQ_ELECTRICAL_OMEGA_RAD_S     314.1592654f

static float DqVectorMagnitudeSquared(float d, float q)
{
    return d * d + q * q;
}

static uint8_t DqLimitVector(float *d, float *q, float limit)
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

#if DQ_CONTROL_MODE == DQ_VOLTAGE_LOOP
static float DqVoltagePiUpdate(DqPiState *pi,
                               float error,
                               float kp,
                               float ki)
{
    float integral_candidate = pi->integral + ki * kp * error;
    float output = kp * error + integral_candidate;

    if (output > DQ_PI_CORRECTION_LIMIT_V)
    {
        if (error < 0.0f)
        {
            pi->integral = integral_candidate;
        }
        return DQ_PI_CORRECTION_LIMIT_V;
    }
    if (output < -DQ_PI_CORRECTION_LIMIT_V)
    {
        if (error > 0.0f)
        {
            pi->integral = integral_candidate;
        }
        return -DQ_PI_CORRECTION_LIMIT_V;
    }

    pi->integral = integral_candidate;
    return output;
}
#endif

#if DQ_CONTROL_MODE == DQ_VOLTAGE_CURRENT_LOOP
static void DqUpdateOuterLoop(DqControl *control,
                              float vd_ref,
                              float vd,
                              float vq)
{
    float error_d = vd_ref - vd;
    float error_q = -vq;
    float candidate_integral_d = control->outer_d_pi.integral
                                 + control->config.outer_ki
                                   * control->config.outer_kp * error_d;
    float candidate_integral_q = control->outer_q_pi.integral
                                 + control->config.outer_ki
                                   * control->config.outer_kp * error_q;
    float candidate_d = control->config.outer_kp * error_d
                        + candidate_integral_d;
    float candidate_q = control->config.outer_kp * error_q
                        + candidate_integral_q;
    float old_d = control->config.outer_kp * error_d
                  + control->outer_d_pi.integral;
    float old_q = control->config.outer_kp * error_q
                  + control->outer_q_pi.integral;
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

    candidate_magnitude_squared =
        DqVectorMagnitudeSquared(candidate_d, candidate_q);
    old_magnitude_squared = DqVectorMagnitudeSquared(old_d, old_q);
    candidate_limited = DqLimitVector(&limited_candidate_d,
                                      &limited_candidate_q,
                                      DQ_CURRENT_REF_LIMIT_A);
    old_limited = DqLimitVector(&limited_old_d,
                                &limited_old_q,
                                DQ_CURRENT_REF_LIMIT_A);

    allow_integration = (uint8_t)((candidate_limited == 0U) ||
                        (candidate_magnitude_squared < old_magnitude_squared));

    integration_step_d = candidate_integral_d
                         - control->outer_d_pi.integral;
    integration_step_q = candidate_integral_q
                         - control->outer_q_pi.integral;
    if ((control->previous_voltage_limited != 0U) &&
        ((integration_step_d * control->voltage_excess_d +
          integration_step_q * control->voltage_excess_q) >= 0.0f))
    {
        allow_integration = 0U;
    }

    if (allow_integration != 0U)
    {
        control->outer_d_pi.integral = candidate_integral_d;
        control->outer_q_pi.integral = candidate_integral_q;
        control->held_id_ref = limited_candidate_d;
        control->held_iq_ref = limited_candidate_q;
        control->current_ref_limited = candidate_limited;
    }
    else
    {
        control->held_id_ref = limited_old_d;
        control->held_iq_ref = limited_old_q;
        control->current_ref_limited = old_limited;
    }
}

static void DqUpdateCurrentLoop(DqControl *control,
                                float vd,
                                float vq,
                                float id,
                                float iq,
                                float vdc,
                                DqControlOutput *output)
{
    float error_d = control->held_id_ref - id;
    float error_q = control->held_iq_ref - iq;

//    float error_d = 0.3f - id;
//    float error_q = 0.0f - iq;

    float candidate_integral_d = control->current_d_pi.integral
                                 + control->config.current_ki
                                   * control->config.current_kp * error_d;
    float candidate_integral_q = control->current_q_pi.integral
                                 + control->config.current_ki
                                   * control->config.current_kp * error_q;
    float candidate_correction_d = control->config.current_kp * error_d
                                   + candidate_integral_d;
    float candidate_correction_q = control->config.current_kp * error_q
                                   + candidate_integral_q;
    float old_correction_d = control->config.current_kp * error_d
                             + control->current_d_pi.integral;
    float old_correction_q = control->config.current_kp * error_q
                             + control->current_q_pi.integral;
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
    uint8_t candidate_voltage_limited;
    uint8_t old_voltage_limited;
    uint8_t allow_integration;

    candidate_correction_raw_squared =
        DqVectorMagnitudeSquared(candidate_correction_d,
                                 candidate_correction_q);
    old_correction_raw_squared =
        DqVectorMagnitudeSquared(old_correction_d, old_correction_q);
    candidate_correction_limited =
        DqLimitVector(&candidate_correction_d,
                      &candidate_correction_q,
                      DQ_CURRENT_CORRECTION_LIMIT_V);
    (void)DqLimitVector(&old_correction_d,
                        &old_correction_q,
                        DQ_CURRENT_CORRECTION_LIMIT_V);

    decoupling_d = -DQ_ELECTRICAL_OMEGA_RAD_S * DQ_FILTER_L_H * iq;
    decoupling_q = DQ_ELECTRICAL_OMEGA_RAD_S * DQ_FILTER_L_H * id;

    candidate_ud = vd + candidate_correction_d + decoupling_d;
    candidate_uq = vq + candidate_correction_q + decoupling_q;
    old_ud = vd + old_correction_d + decoupling_d;
    old_uq = vq + old_correction_q + decoupling_q;
    candidate_voltage_raw_squared =
        DqVectorMagnitudeSquared(candidate_ud, candidate_uq);
    old_voltage_raw_squared = DqVectorMagnitudeSquared(old_ud, old_uq);

    vector_limit = DQ_VOLTAGE_UTILIZATION * vdc
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
        control->current_d_pi.integral = candidate_integral_d;
        control->current_q_pi.integral = candidate_integral_q;
        output->ud = limited_candidate_ud;
        output->uq = limited_candidate_uq;
        output->voltage_limited = candidate_voltage_limited;
        control->voltage_excess_d = candidate_ud - limited_candidate_ud;
        control->voltage_excess_q = candidate_uq - limited_candidate_uq;
    }
    else
    {
        output->ud = limited_old_ud;
        output->uq = limited_old_uq;
        output->voltage_limited = old_voltage_limited;
        control->voltage_excess_d = old_ud - limited_old_ud;
        control->voltage_excess_q = old_uq - limited_old_uq;
    }

    control->previous_voltage_limited = output->voltage_limited;
}
#endif

void DqControl_Init(DqControl *control,
                    const InverterConfig *config)
{
    if (control == NULL)
    {
        return;
    }

    memset(control, 0, sizeof(*control));
    if (config != NULL)
    {
        control->config = *config;
    }
}

void DqControl_Reset(DqControl *control)
{
    InverterConfig config;

    if (control == NULL)
    {
        return;
    }

    config = control->config;
    memset(control, 0, sizeof(*control));
    control->config = config;
}

void DqControl_SetConfig(DqControl *control,
                         const InverterConfig *config)
{
    if ((control != NULL) && (config != NULL))
    {
        control->config = *config;
    }
}

void DqControl_Step(DqControl *control,
                    const DqControlInput *input,
                    DqControlOutput *output)
{
    float sin_sample;
    float cos_sample;
    float v_alpha;
    float v_beta;
    float i_alpha;
    float i_beta;

    if ((control == NULL) || (input == NULL) || (output == NULL))
    {
        return;
    }

    sin_sample = input->sin_theta;
    cos_sample = input->cos_theta;
#if DQ_CONTROL_MODE == DQ_VOLTAGE_CURRENT_LOOP
    sin_sample = input->sin_theta * DQ_SAMPLE_HALF_STEP_COS
                 - input->cos_theta * DQ_SAMPLE_HALF_STEP_SIN;
    cos_sample = input->cos_theta * DQ_SAMPLE_HALF_STEP_COS
                 + input->sin_theta * DQ_SAMPLE_HALF_STEP_SIN;
#endif

    v_alpha = input->u_u;
    v_beta = input->u_vw * DQ_INV_SQRT_THREE;
    i_alpha = input->iu;
    i_beta = (input->iv - input->iw) * DQ_INV_SQRT_THREE;

    output->vd = v_alpha * cos_sample + v_beta * sin_sample;
    output->vq = -v_alpha * sin_sample + v_beta * cos_sample;
    output->id = i_alpha * cos_sample + i_beta * sin_sample;
    output->iq = -i_alpha * sin_sample + i_beta * cos_sample;
    output->id_ref = 0.0f;
    output->iq_ref = 0.0f;
    output->current_ref_limited = 0U;
    output->voltage_limited = 0U;

#if DQ_CONTROL_MODE == DQ_OPEN_LOOP
    output->ud = input->vd_ref;
    output->uq = 0.0f;
#elif DQ_CONTROL_MODE == DQ_VOLTAGE_LOOP
    output->ud = input->vd_ref
                 + DqVoltagePiUpdate(&control->voltage_d_pi,
                                     input->vd_ref - output->vd,
                                     control->config.voltage_kp,
                                     control->config.voltage_ki);
    output->uq = DqVoltagePiUpdate(&control->voltage_q_pi,
                                   -output->vq,
                                   control->config.voltage_kp,
                                   control->config.voltage_ki);
#else
    if (control->outer_divider == 0U)
    {
        DqUpdateOuterLoop(control, input->vd_ref, output->vd, output->vq);
        control->outer_divider = DQ_OUTER_DIVIDER_RELOAD;
    }
    else
    {
        control->outer_divider--;
    }

    output->id_ref = control->held_id_ref;
    output->iq_ref = control->held_iq_ref;
    output->current_ref_limited = control->current_ref_limited;
    DqUpdateCurrentLoop(control,
                        output->vd,
                        output->vq,
                        output->id,
                        output->iq,
                        input->vdc,
                        output);
#endif

#if DQ_CONTROL_MODE != DQ_VOLTAGE_CURRENT_LOOP
    output->voltage_limited = DqLimitVector(
        &output->ud,
        &output->uq,
        DQ_VOLTAGE_UTILIZATION * input->vdc
        * DQ_INV_SQRT_THREE);
#endif
}
