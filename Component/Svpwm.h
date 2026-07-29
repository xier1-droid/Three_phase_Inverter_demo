#ifndef SVPWM_H
#define SVPWM_H

#define SVPWM_MODE_MIN_MAX_ZERO_SEQUENCE  0U
#define SVPWM_MODE_THIRD_HARMONIC         1U

#ifndef SVPWM_MODULATION_MODE
#define SVPWM_MODULATION_MODE SVPWM_MODE_THIRD_HARMONIC
#endif

#if ((SVPWM_MODULATION_MODE != SVPWM_MODE_MIN_MAX_ZERO_SEQUENCE) && \
     (SVPWM_MODULATION_MODE != SVPWM_MODE_THIRD_HARMONIC))
#error "Invalid SVPWM_MODULATION_MODE"
#endif

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
} SvpwmDuty;

void Svpwm_Calculate(float ud,
                     float uq,
                     float sin_theta,
                     float cos_theta,
                     float vdc,
                     SvpwmDuty *duty);

#endif
