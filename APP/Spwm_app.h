#ifndef __SPWM_APP_H
#define __SPWM_APP_H

#include "mydefine.h"

float C_cal_rms(float x);
float V_cal_rms(float x);

extern volatile uint8_t wave_enable_tim1;
extern volatile uint8_t wave_enable_tim8;

typedef struct
{
    float vll_ref_rms;  /* User reference: line-to-line RMS volts. */
    float vd_ref;       /* Ramped d-axis phase-voltage peak reference. */
    float vd;           /* Measured d-axis phase-voltage peak value. */
    float vq;           /* Measured q-axis phase-voltage peak value. */
    float ud_cmd;       /* Limited inverter d-axis voltage command. */
    float uq_cmd;       /* Limited inverter q-axis voltage command. */
    float kp;           /* Shared proportional gain for both axes. */
    float ki;           /* Shared integral gain in 1/s. */
} InverterVoltageStatus;

void Inverter_Wave_Start_TIM1(void);
void Inverter_Wave_Stop_TIM1(void);
void Inverter_Wave_Start_TIM8(void);
void Inverter_Wave_Stop_TIM8(void);
void Inverter_Start(void);
void Inverter_Stop(void);
void Inverter_ClearFault(void);

/* Runtime voltage-loop tuning and diagnostic interface. */
void Inverter_SetVoltageKp(float kp);
void Inverter_SetVoltageKi(float ki);
void Inverter_SetLineVoltageRef(float vll_rms);
float Inverter_GetVoltageKp(void);
float Inverter_GetVoltageKi(void);
float Inverter_GetLineVoltageRef(void);
void Inverter_GetVoltageStatus(InverterVoltageStatus *status);

#endif
