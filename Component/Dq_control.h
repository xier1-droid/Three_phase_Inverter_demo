#ifndef DQ_CONTROL_H
#define DQ_CONTROL_H

#include <stdint.h>

#define DQ_OPEN_LOOP                 0U
#define DQ_VOLTAGE_LOOP              1U
#define DQ_VOLTAGE_CURRENT_LOOP      2U

#ifndef DQ_CONTROL_MODE
#define DQ_CONTROL_MODE              DQ_VOLTAGE_CURRENT_LOOP
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
#define DQ_VOLTAGE_KP_MAX                 2.0f
#define DQ_VOLTAGE_KI_MAX                 500.0f
#define DQ_OUTER_KP_MAX                   2.0f
#define DQ_OUTER_KI_MAX                   2.0f
#define DQ_CURRENT_KP_MAX                 20.0f
#define DQ_CURRENT_KI_MAX                 2.0f
#define DQ_LINE_VOLTAGE_REF_MAX_V         32.0f

typedef struct
{
    float vll_ref_rms;
    float voltage_kp;
    float voltage_ki;
    float outer_kp;
    float outer_ki;
    float current_kp;
    float current_ki;
    float vdc;
} InverterConfig;

typedef struct
{
    float integral;
} DqPiState;

typedef struct
{
    InverterConfig config;
    DqPiState voltage_d_pi;
    DqPiState voltage_q_pi;
    DqPiState outer_d_pi;
    DqPiState outer_q_pi;
    DqPiState current_d_pi;
    DqPiState current_q_pi;
    float held_id_ref;
    float held_iq_ref;
    float voltage_excess_d;
    float voltage_excess_q;
    uint8_t outer_divider;
    uint8_t current_ref_limited;
    uint8_t previous_voltage_limited;
} DqControl;

typedef struct
{
    float vd_ref;
    float u_u;
    float u_vw;
    float iu;
    float iv;
    float iw;
    float sin_theta;
    float cos_theta;
} DqControlInput;

typedef struct
{
    float vd;
    float vq;
    float id_ref;
    float id;
    float iq_ref;
    float iq;
    float ud;
    float uq;
    uint8_t current_ref_limited;
    uint8_t voltage_limited;
} DqControlOutput;

void DqControl_Init(DqControl *control,
                    const InverterConfig *config);
void DqControl_Reset(DqControl *control);
void DqControl_SetConfig(DqControl *control,
                         const InverterConfig *config);
void DqControl_Step(DqControl *control,
                    const DqControlInput *input,
                    DqControlOutput *output);

#endif
