#include "Dq_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DQ_PI_CORRECTION_LIMIT_V      15.0f

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

    if ((control == NULL) || (input == NULL) || (output == NULL))
    {
        return;
    }

    sin_sample = input->sin_theta;
    cos_sample = input->cos_theta;

    v_alpha = input->u_u;
    v_beta = input->u_vw * DQ_INV_SQRT_THREE;

    output->vd = v_alpha * cos_sample + v_beta * sin_sample;
    output->vq = -v_alpha * sin_sample + v_beta * cos_sample;
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
#endif

    output->voltage_d_integral = control->voltage_d_pi.integral;
    output->voltage_q_integral = control->voltage_q_pi.integral;

    output->voltage_limited = DqLimitVector(
        &output->ud,
        &output->uq,
        DQ_VOLTAGE_UTILIZATION * input->vdc
        * DQ_INV_SQRT_THREE);
}
