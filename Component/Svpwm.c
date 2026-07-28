#include "Svpwm.h"

#include <math.h>
#include <stddef.h>

#define SVPWM_DUTY_MIN 0.05f
#define SVPWM_DUTY_MAX 0.95f

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
    float u_max;
    float u_min;
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

    u_max = fmaxf(ua, fmaxf(ub, uc));
    u_min = fminf(ua, fminf(ub, uc));
    u_zero = -0.5f * (u_max + u_min);

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
