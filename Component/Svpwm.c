#include "Svpwm.h"

#include <math.h>
#include <stddef.h>

#define SVPWM_DUTY_MIN 0.05f
#define SVPWM_DUTY_MAX 0.95f
#define SVPWM_VECTOR_MAGNITUDE_SQUARED_EPSILON 1.0e-12f

static float Svpwm_Clamp(float value, float minimum, float maximum)
{
    return fminf(fmaxf(value, minimum), maximum);
}

void Svpwm_Calculate(float ud,
                     float uq,
                     float sin_theta,
                     float cos_theta,
                     float vdc,
                     SvpwmDuty *duty)
{
    float u_alpha;
    float u_beta;
    float ua;
    float ub;
    float uc;
#if SVPWM_MODULATION_MODE == SVPWM_MODE_MIN_MAX_ZERO_SEQUENCE
    float u_max;
    float u_min;
#else
    float magnitude_squared;
#endif
    float u_zero;

    if (duty == NULL)
    {
        return;
    }

    if (vdc <= 0.0f)
    {
        duty->duty_a = 0.5f;
        duty->duty_b = 0.5f;
        duty->duty_c = 0.5f;
        return;
    }

    u_alpha = ud * cos_theta - uq * sin_theta;
    u_beta = ud * sin_theta + uq * cos_theta;

    ua = u_alpha;
    ub = -0.5f * u_alpha + 0.8660254f * u_beta;
    uc = -0.5f * u_alpha - 0.8660254f * u_beta;

#if SVPWM_MODULATION_MODE == SVPWM_MODE_MIN_MAX_ZERO_SEQUENCE
    u_max = fmaxf(ua, fmaxf(ub, uc));
    u_min = fminf(ua, fminf(ub, uc));
    u_zero = -0.5f * (u_max + u_min);
#else
    magnitude_squared = u_alpha * u_alpha + u_beta * u_beta;
    if (magnitude_squared > SVPWM_VECTOR_MAGNITUDE_SQUARED_EPSILON)
    {
        u_zero = -(u_alpha * (u_alpha * u_alpha
                              - 3.0f * u_beta * u_beta))
                 / (6.0f * magnitude_squared);
    }
    else
    {
        u_zero = 0.0f;
    }
#endif

    duty->duty_a = Svpwm_Clamp(0.5f + (ua + u_zero) / vdc,
                               SVPWM_DUTY_MIN,
                               SVPWM_DUTY_MAX);
    duty->duty_b = Svpwm_Clamp(0.5f + (ub + u_zero) / vdc,
                               SVPWM_DUTY_MIN,
                               SVPWM_DUTY_MAX);
    duty->duty_c = Svpwm_Clamp(0.5f + (uc + u_zero) / vdc,
                               SVPWM_DUTY_MIN,
                               SVPWM_DUTY_MAX);
}
