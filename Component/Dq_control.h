#ifndef DQ_CONTROL_H
#define DQ_CONTROL_H

#include <stdint.h>

#define DQ_OPEN_LOOP                 0U
#define DQ_VOLTAGE_LOOP              1U

#ifndef DQ_CONTROL_MODE
#define DQ_CONTROL_MODE              DQ_VOLTAGE_LOOP
#endif

#if ((DQ_CONTROL_MODE != DQ_OPEN_LOOP) && \
     (DQ_CONTROL_MODE != DQ_VOLTAGE_LOOP))
#error "Invalid DQ_CONTROL_MODE"
#endif

#define DQ_INV_SQRT_THREE                 0.577350269f
#define DQ_VOLTAGE_UTILIZATION            0.9f
#define DQ_VOLTAGE_KP_MAX                 2.0f
#define DQ_VOLTAGE_KI_MAX                 500.0f
#define DQ_LINE_VOLTAGE_REF_MAX_V         34.0f

typedef struct
{
    float vll_ref_rms;
    float voltage_kp;
    float voltage_ki;
    float voltage_compensation_offset_v;
    float voltage_compensation_slope_v_per_a;
    uint8_t voltage_compensation_enabled;
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
} DqControl;

typedef struct
{
    float vd_ref;
    float vdc;
    float u_u;
    float u_vw;
    float sin_theta;
    float cos_theta;
} DqControlInput;

typedef struct
{
    float vd;
    float vq;
    float voltage_d_integral;
    float voltage_q_integral;
    float ud;
    float uq;
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
