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

extern TIM_HandleTypeDef htim8;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc2;

extern DAC_HandleTypeDef hdac;
	
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;



#endif

