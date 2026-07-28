#ifndef DQ_CONTROL_H
#define DQ_CONTROL_H

#include <stdint.h>

#define DQ_OPEN_LOOP                 0U
#define DQ_VOLTAGE_LOOP              1U
#define DQ_VOLTAGE_CURRENT_LOOP      2U

#ifndef DQ_CONTROL_MODE
#define DQ_CONTROL_MODE              DQ_VOLTAGE_LOOP
#endif

#if ((DQ_CONTROL_MODE != DQ_OPEN_LOOP) && \
     (DQ_CONTROL_MODE != DQ_VOLTAGE_LOOP) && \
     (DQ_CONTROL_MODE != DQ_VOLTAGE_CURRENT_LOOP))
#error "Invalid DQ_CONTROL_MODE"
#endif

#define DQ_INV_SQRT_THREE                 0.577350269f
#define DQ_FILTER_L_H                     0.001f
#define DQ_CURRENT_REF_LIMIT_A            3.5f
#define DQ_CURRENT_CORRECTION_LIMIT_V     10.0f
#define DQ_VOLTAGE_UTILIZATION            0.9f
#define DQ_OUTER_KP_MAX                   2.0f
#define DQ_OUTER_KI_MAX                   2.0f
#define DQ_CURRENT_KP_MAX                 20.0f
#define DQ_CURRENT_KI_MAX                 2.0f

typedef struct
{
    float vd_ref;
    float vq_ref;
    float vd;
    float vq;
    float id;
    float iq;
    float vdc;
    float omega_rad_s;
} DqCascadeInput;

typedef struct
{
    float id_ref;
    float iq_ref;
    float ud_cmd;
    float uq_cmd;
    uint8_t current_ref_limited;
    uint8_t current_correction_limited;
    uint8_t voltage_limited;
} DqCascadeOutput;

typedef struct
{
    float vdc;
    float vd_ref;
    float vd;
    float vq;
    float id_ref;
    float id;
    float iq_ref;
    float iq;
    float ud_cmd;
    float uq_cmd;
    float outer_kp;
    float outer_ki;
    float current_kp;
    float current_ki;
    uint8_t current_ref_limited;
    uint8_t current_correction_limited;
    uint8_t voltage_limited;
} DqCascadeStatus;

void DqCascade_Reset(void);
void DqCascade_Step(const DqCascadeInput *input,
                    DqCascadeOutput *output);

void DqCascade_SetOuterKp(float value);
void DqCascade_SetOuterKi(float value);
void DqCascade_SetCurrentKp(float value);
void DqCascade_SetCurrentKi(float value);

float DqCascade_GetOuterKp(void);
float DqCascade_GetOuterKi(void);
float DqCascade_GetCurrentKp(void);
float DqCascade_GetCurrentKi(void);
void DqCascade_GetStatus(DqCascadeStatus *status);

#endif
