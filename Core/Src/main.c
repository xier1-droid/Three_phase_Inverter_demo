/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mydefine.h" // ȫ�ֶ���ͷ�ļ�
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

#define RMS_FILTER_LEN  256
#define pi 3.1415926f
#define sqrt_2 1.414213562f
// 缓冲区
float Voltage_Rms_Buffer[RMS_FILTER_LEN];
float Current_Rms_Buffer[RMS_FILTER_LEN];
extern MovingAverageFilter_t voltage_filter;
extern MovingAverageFilter_t current_filter;
extern LowPassFilter_t LPF_Current;
extern PID_T PID_Voltage;
extern PID_T PID_Current;
extern PR_T PR_Current;
extern PI_TypeDef PI_C;
extern float PID_OUT_C,Iref_inst;
extern SOGI_PLL_T Grid_PLL;

extern uint16_t i;
//交直流偏置
extern volatile uint32_t adc_val_buffer[];
extern volatile uint32_t adc_val_buffer_2[];
extern LowPassFilter_t Voltage_Offset_Filter;
extern LowPassFilter_t Current_Offset_Filter;
extern LowPassFilter_t Voltage_Offset_Filter_2;
extern LowPassFilter_t Current_Offset_Filter_2;


float P_v=3.8f,I_v=0.01f,target_v=24.0f;//1.65 0.028
float P_c=0.08f,I_c=0.03f,target_c=0.5f * sqrt_2;		//电流幅值
float PR_p_c=0.03f,PR_r_c=6.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_DAC_Init();
  MX_TIM13_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
//	Delay_Init();
//	OLED_I2C_Init();
//	OLED_Init();
	Uart_init();
	MovingAverage_Init(&voltage_filter, Voltage_Rms_Buffer, RMS_FILTER_LEN);
	MovingAverage_Init(&current_filter, Current_Rms_Buffer, RMS_FILTER_LEN);
	LowPass_Init(&LPF_Current,0.5 , 0);
	pid_init(&PID_Voltage, P_v, I_v , 0 , 0, 1);
	pid_init(&PID_Current, P_c, I_c , 0 , 0, 1);
	pr_init(&PR_Current, PR_p_c, PR_r_c, 2*pi*50.0f, 3,5e-5f);
	f32_PI_Init(&PI_C ,5e-5f , 0.3f, 0.2f, 1, -1);
	PR_Precompute(&PR_Current);
	sogi_pll_init(&Grid_PLL, 1.0f, 50.0f, 5e-5f, 1.0f, 10.0f, 2.0f*pi*8.0f);
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val_buffer, 2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_val_buffer_2, 2) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT | DMA_IT_TC);
	__HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_HT | DMA_IT_TC);
	if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
	if (HAL_TIM_Base_Start_IT(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	HAL_Delay(2);

	//�ϵ�ֱ��ƫ��У׼: PWM/�ж���δ����,ADC���ɱ���ֱ������,�ɿ��������ֵ��ƽ��
	{
		#define ADC_OFFSET_CALIB_SAMPLES 256
		uint32_t v_sum = 0, c_sum = 0;
		uint32_t v_sum_2 = 0, c_sum_2 = 0;
		for (uint16_t n = 0; n < ADC_OFFSET_CALIB_SAMPLES; n++)
		{
			v_sum += adc_val_buffer[1];
			c_sum += adc_val_buffer[0];
			v_sum_2 += adc_val_buffer_2[1];
			c_sum_2 += adc_val_buffer_2[0];
			HAL_Delay(1);
		}
		float v_offset_init = (float)v_sum / ADC_OFFSET_CALIB_SAMPLES;
		float c_offset_init = (float)c_sum / ADC_OFFSET_CALIB_SAMPLES;
		float v_offset_init_2 = (float)v_sum_2 / ADC_OFFSET_CALIB_SAMPLES;
		float c_offset_init_2 = (float)c_sum_2 / ADC_OFFSET_CALIB_SAMPLES;
		LowPass_Init(&Voltage_Offset_Filter, 0.9999f, v_offset_init);
		LowPass_Init(&Current_Offset_Filter, 0.9999f, c_offset_init);
		LowPass_Init(&Voltage_Offset_Filter_2, 0.9999f, v_offset_init_2);
		LowPass_Init(&Current_Offset_Filter_2, 0.9999f, c_offset_init_2);
	}
	
	HAL_TIM_Base_Start_IT(&htim13);
//  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);// 启动定时器8 通道1
//  HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);//启动定时器8 通道1的互补通道
//	 HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);// 启动定时器8 通道1
//  HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);//启动定时器8 通道1的互补通道
//  __HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,4200);// 设置比较值 
//	__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,4200);// 设置比较值
//	
//  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);// 启动定时器1 通道1
//  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);//启动定时器1 通道1的互补通道
//	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);// 启动定时器1 通道1
//  HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);//启动定时器1 通道1的互补通道
//  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,4200);// 设置比较值 
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,4200);// 设置比较值
	
  HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,2048);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_2);
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,2048);
	
	scheduler_init();
	printf("ok\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		scheduler_run();
//		printf("{i}%d\r\n	",i);
//		printf("{Voltage_Get}%.2f\r\n	",Voltage_Get);
//		printf("{ADC_v_val}:%.2f\r\n",ADC_v_val);
//		printf("{Voltage_val}:%.2f\r\n",Voltage_val);
//		printf("{Voltage_Rms}:%.2f\r\n",Voltage_Rms);
//		printf("{Voltage_Rms_Filtered}:%.2f\r\n",Voltage_Rms_Filtered);
//		printf("{target_vrmscurrent_vrms}%.2f,%.2f\r\n",target_v ,Voltage_Rms_Filtered);
		
//		printf("{Current_Get}%.2f\r\n	",Current_Get);
//		printf("{ADC_c_val}:%.2f\r\n",ADC_c_val);
//		printf("{Current_val}:%.2f\r\n",Current_val);
//		printf("{Current_Rms}:%.2f\r\n",Current_Rms);
//		printf("{PID_ADJUST_Current}%.2f,%.2f\r\n",target_c/sqrtf(2),Current_Rms);
//		printf("{sine_norm}%.2f,%.2f\r\n",PID_OUT_C,Iref_inst);
//*****************************************************************************//		
//		printf("{Voltage_Get_2}:%.2f\r\n	",Voltage_Get_2);
//		printf("{ADC_v_val_2}:%.2f\r\n",ADC_v_val_2);
//		printf("{Voltage_val_2}:%.2f\r\n",Voltage_val_2);
//		printf("{Voltage_Rms_2}:%.2f\r\n",Voltage_Rms_2);
//		printf("{Voltage_Rms_Filtered}:%.2f\r\n",Voltage_Rms_Filtered);
		
//		printf("{Current_Get_2}:%.2f\r\n	",Current_Get_2);
//		printf("{ADC_c_val_2}:%.2f\r\n",ADC_c_val_2);
//		printf("{Current_val_2}:%.2f\r\n",Current_val_2);
		
//		printf("{q_f}%.2f,%.2f\r\n	",Grid_PLL.freq_hz,Grid_PLL.v_q);
//		printf("{sample_theta}%.2f,%.2f\r\n	",Grid_PLL.v_alpha ,Grid_PLL.theta);
	//*****************************************************************************//
//		printf("{Three_phase_line_U}:%.2f,%.2f,%.2f\r\n",U_uv,U_vw,U_uw);
//		printf("{d_q_feedback}:%.2f,%.2f\r\n",dq_vd_feedback,dq_vq_feedback);
    /* USER CO0DE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,1000);
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
