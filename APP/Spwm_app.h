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
    INVERTER_PARAMETER_OUTER_KP,
    INVERTER_PARAMETER_OUTER_KI,
    INVERTER_PARAMETER_CURRENT_KP,
    INVERTER_PARAMETER_CURRENT_KI
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
    float id_ref;
    float id;
    float iq_ref;
    float iq;
    float ud;
    float uq;
    uint8_t current_ref_limited;
    uint8_t voltage_limited;
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
