#ifndef __MYDEFINE_H
#define __MYDEFINE_H

#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include "stdint.h"
#include "stdlib.h"
#include <stdbool.h>
#include <math.h>

#include "usart.h"
#include "main.h"

#include "scheduler.h"
#include "Uart_cmp.h"
#include "Oled_cmp.h"
#include "Key_cmp.h"
#include "Adc_cmp.h"
#include "filter.h"
#include "Pid.h"
#include "Pr.h"
#include "Sogi_pll.h"

#include "Oled_app.h"
#include "Spwm_app.h"

extern float Voltage_Rms;
extern float Voltage_Rms_2;
extern float Current_Rms;

extern float Voltage_val;
extern float Current_val;
extern float Voltage_val_2;
extern float Current_val_2;

extern float Voltage_Get;
extern float Current_Get;
extern float Voltage_Get_2;
extern float Current_Get_2;
extern float ADC_v_val ,ADC_c_val;
extern float ADC_v_val_2 ,ADC_c_val_2;

extern float Voltage_Rms_Filtered;
extern float target_v;
extern float target_c;
extern float P_v,I_v;

extern float Voltage_Rms_Filtered;
extern float Current_Rms_Filtered;

//线电压
extern float U_uv,U_vw,U_uw;
//相电压
extern float U_u,U_v,U_w;
//相电流
extern float Three_phase_I_u,Three_phase_I_w,Three_phase_I_v;
//反馈Park变换
extern volatile float dq_vd_feedback;
extern volatile float dq_vq_feedback;

extern TIM_HandleTypeDef htim8;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;

extern DAC_HandleTypeDef hdac;
	
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;



#endif

