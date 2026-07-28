#ifndef SVPWM_H
#define SVPWM_H

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
