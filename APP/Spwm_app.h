#ifndef __SPWM_APP_H
#define __SPWM_APP_H

#include "mydefine.h"
#include "Dq_control.h"

extern volatile uint8_t wave_enable_tim8;

typedef enum
{
    INVERTER_PARAMETER_VLL_REF_RMS = 0,
    INVERTER_PARAMETER_VOLTAGE_KP,
    INVERTER_PARAMETER_VOLTAGE_KI,
    INVERTER_PARAMETER_VCOMP_ENABLE,
    INVERTER_PARAMETER_VCOMP_OFFSET,
    INVERTER_PARAMETER_VCOMP_SLOPE
} InverterParameter;

typedef struct
{
    uint8_t mode;
    uint8_t running;
    uint8_t fault_latched;
    float vdc;
    float vd_ref;
    float vd;
    float vq;
    float vd_cycle_average;
    float vq_cycle_average;
    float u_uv_cycle_mean_square;
    float u_vw_cycle_mean_square;
    float u_wu_cycle_mean_square;
    float iu_cycle_mean_square;
    float iv_cycle_mean_square;
    float iw_cycle_mean_square;
    float load_current_rms;
    float voltage_compensation_target_v;
    float voltage_compensation_applied_v;
    float effective_vll_ref_rms;
    float voltage_d_integral;
    float voltage_q_integral;
    float ud;
    float uq;
    uint8_t voltage_limited;
    uint8_t cycle_diagnostic_valid;
    InverterConfig config;
} InverterStatus;

void Inverter_Init(void);
bool Inverter_SetParameter(InverterParameter parameter, float value);
void Inverter_GetStatus(InverterStatus *status);

void Inverter_Wave_Start_TIM8(void);
void Inverter_Wave_Stop_TIM8(void);
void Inverter_Start(void);
void Inverter_Stop(void);
void Inverter_ClearFault(void);

#endif
