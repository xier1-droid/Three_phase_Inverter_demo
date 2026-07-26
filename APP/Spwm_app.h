#ifndef __SPWM_APP_H
#define __SPWM_APP_H

#include "mydefine.h"

float C_cal_rms(float x);
float V_cal_rms(float x);

extern volatile uint8_t wave_enable_tim1;
extern volatile uint8_t wave_enable_tim8;
void Inverter_Wave_Start_TIM1(void);
void Inverter_Wave_Stop_TIM1(void);
void Inverter_Wave_Start_TIM8(void);
void Inverter_Wave_Stop_TIM8(void);
void Inverter_Start(void);
void Inverter_Stop(void);
void Inverter_ClearFault(void);

#endif
