#include "Spwm_app.h"
#include "Inverter_sampling.h"
#include "Svpwm.h"

#define DQ_SQRT_TWO_THIRDS 0.816496581f
#define DQ_VDC_NOMINAL_V 60.0f
#define CURRENT_TRIP_A 5.0f
#define OVERCURRENT_TRIP_COUNT 3U

static float soft_start_ratio = 0.0f;      // 0.0 ~ 1.0，当前软启动系数
#define SOFT_START_CYCLES   50.0f   // 软启动持续的基波周期数（50Hz下 20周期≈0.4s）
#define SOFT_START_STEP     (1.0f / (SOFT_START_CYCLES * 400.0f))  // 每次TIM8中断（每周期400点）的增量

volatile uint8_t wave_enable_tim8 = 0;   // 0=未发波（上电默认），1=已发波
// ===== 设备状态 + 启停控制 + 参数读写 =====
static volatile uint8_t fault_latched = 0;
static volatile uint8_t stop_requested = 0;
static uint8_t overcurrent_count = 0;
static uint8_t tim8_pwm_channels_started = 0;
static uint16_t count = 0U;
static float theta = 0.0f;
static InverterConfig inverter_config;
static InverterStatus inverter_status;
static DqControl dq_control;
static InverterMeasurements measurements;

static void Inverter_ResetControlState(void)
{
    DqControl_Reset(&dq_control);
    inverter_status.vd_ref = 0.0f;
    inverter_status.vd = 0.0f;
    inverter_status.vq = 0.0f;
    inverter_status.id_ref = 0.0f;
    inverter_status.id = 0.0f;
    inverter_status.iq_ref = 0.0f;
    inverter_status.iq = 0.0f;
    inverter_status.ud = 0.0f;
    inverter_status.uq = 0.0f;
    inverter_status.current_ref_limited = 0U;
    inverter_status.voltage_limited = 0U;
}

void Inverter_Init(void)
{
    memset(&inverter_status, 0, sizeof(inverter_status));
    memset(&measurements, 0, sizeof(measurements));

    inverter_config.vll_ref_rms = 30.6f;
    inverter_config.voltage_kp = 0.035f;
    inverter_config.voltage_ki = 0.006f;
    inverter_config.outer_kp = 0.0f;
    inverter_config.outer_ki = 0.0f;
    inverter_config.current_kp = 0.0f;
    inverter_config.current_ki = 0.0f;
    inverter_config.vdc = DQ_VDC_NOMINAL_V;

    DqControl_Init(&dq_control, &inverter_config);
    inverter_status.mode = DQ_CONTROL_MODE;
    inverter_status.config = inverter_config;
}

bool Inverter_SetParameter(InverterParameter parameter, float value)
{
    InverterConfig new_config = inverter_config;
    uint32_t primask;
    uint8_t reset_required = 0U;

    switch (parameter)
    {
        case INVERTER_PARAMETER_VLL_REF_RMS:
            if (!((value >= 0.0f) &&
                  (value <= DQ_LINE_VOLTAGE_REF_MAX_V)))
            {
                return false;
            }
            new_config.vll_ref_rms = value;
            break;

        case INVERTER_PARAMETER_VOLTAGE_KP:
            if (!((value >= 0.0f) && (value <= DQ_VOLTAGE_KP_MAX)))
            {
                return false;
            }
            new_config.voltage_kp = value;
            break;

        case INVERTER_PARAMETER_VOLTAGE_KI:
            if (!((value >= 0.0f) && (value <= DQ_VOLTAGE_KI_MAX)))
            {
                return false;
            }
            new_config.voltage_ki = value;
            break;

        case INVERTER_PARAMETER_OUTER_KP:
            if ((wave_enable_tim8 != 0U) ||
                !((value >= 0.0f) && (value <= DQ_OUTER_KP_MAX)))
            {
                return false;
            }
            new_config.outer_kp = value;
            reset_required = 1U;
            break;

        case INVERTER_PARAMETER_OUTER_KI:
            if ((wave_enable_tim8 != 0U) ||
                !((value >= 0.0f) && (value <= DQ_OUTER_KI_MAX)))
            {
                return false;
            }
            new_config.outer_ki = value;
            reset_required = 1U;
            break;

        case INVERTER_PARAMETER_CURRENT_KP:
            if ((wave_enable_tim8 != 0U) ||
                !((value >= 0.0f) && (value <= DQ_CURRENT_KP_MAX)))
            {
                return false;
            }
            new_config.current_kp = value;
            reset_required = 1U;
            break;

        case INVERTER_PARAMETER_CURRENT_KI:
            if ((wave_enable_tim8 != 0U) ||
                !((value >= 0.0f) && (value <= DQ_CURRENT_KI_MAX)))
            {
                return false;
            }
            new_config.current_ki = value;
            reset_required = 1U;
            break;

        default:
            return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    inverter_config = new_config;
    DqControl_SetConfig(&dq_control, &inverter_config);
    inverter_status.config = inverter_config;
    if (reset_required != 0U)
    {
        Inverter_ResetControlState();
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

void Inverter_GetStatus(InverterStatus *status)
{
    uint32_t primask;

    if (status == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *status = inverter_status;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Inverter_Wave_Start_TIM8(void)
{
    Inverter_ResetControlState();
    soft_start_ratio = 0.0f;   // 从0开始软启动斜坡

    /* HAL marks PWM channels BUSY, so start them only once. */
    if (!tim8_pwm_channels_started)
    {
        HAL_StatusTypeDef ch1_status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
        HAL_StatusTypeDef ch1n_status = HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
        HAL_StatusTypeDef ch2_status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
        HAL_StatusTypeDef ch2n_status = HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
        HAL_StatusTypeDef ch3_status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
        HAL_StatusTypeDef ch3n_status = HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

        if ((ch1_status != HAL_OK) || (ch1n_status != HAL_OK) ||
            (ch2_status != HAL_OK) || (ch2n_status != HAL_OK) ||
            (ch3_status != HAL_OK) || (ch3n_status != HAL_OK))
        {
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
            wave_enable_tim8 = 0;
            inverter_status.running = 0U;
            return;
        }

        tim8_pwm_channels_started = 1;
    }

    /* A logical stop only clears MOE; restart by enabling MOE again. */
    __HAL_TIM_MOE_ENABLE(&htim8);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 4200);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 4200);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 4200);
	
	    wave_enable_tim8 = 1;
        inverter_status.running = 1U;
}

void Inverter_Wave_Stop_TIM8(void)
{
    Inverter_ResetControlState();
    wave_enable_tim8 = 0;
    inverter_status.running = 0U;

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 4200);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 4200);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 4200);

    /* Disable PWM outputs, but keep TIM8 counter and update IRQ running. */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    soft_start_ratio = 0.0f;   // 重置软启动斜坡
}

// 逆变器 状态控制接口(文件：Uart_cmp.c) 仅软件启停，不直接操作硬件
void Inverter_Start(void)
{
    if (!fault_latched && !wave_enable_tim8)
    {
			soft_start_ratio = 0.0f;
			stop_requested = 0;
			overcurrent_count = 0;
			Inverter_Wave_Start_TIM8();
//			my_printf(&huart1,"System RAMP_UP");	
    }
}

void Inverter_Stop(void)
{
    if (wave_enable_tim8 && !fault_latched)
    {
      stop_requested = 1;   // 置停止请求标志，等待斜坡降频停机
//			my_printf(&huart1,"System Stop_Start");
    }
}

void Inverter_ClearFault(void)
{
    if (fault_latched)
    {
        overcurrent_count = 0;
        fault_latched = 0;
        inverter_status.fault_latched = 0U;
        Inverter_ResetControlState();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM8)  
  {
		
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((Grid_PLL.v_alpha+15.0f)/30.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((Grid_PLL.v_beta+15.0f)/30.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((sinf(Grid_PLL.theta)+1.5f)/3.0f)*4096));
//	  HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,Voltage_Get);
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((ADC_v_val+1.75f)/3.3f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((sine_wave[i]+4300.0f)/8800.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((ADC_c_val_2+1.75f)/3.3f)*4096));
					
		InverterSampling_Update(&measurements);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
		                 (uint32_t)(((measurements.iu + 3.0f) / 6.0f)
		                            * 4096.0f));

//		Current_val = (Current_Get * 3.3f/4096.0f) * 4.96f - 7.956f;      // 电流信号标定(A)//4.96
//		Voltage_val = (Voltage_Get * 3.3f/4096.0f) * 33.125f - 53.66f;  // 电压信号标定(V)
		
//		Voltage_Rms = V_cal_rms(Voltage_val);
//		Voltage_Rms_Filtered = MovingAverage_Update(&voltage_filter,Voltage_Rms);

//		Current_val = LowPass_Update(&LPF_Current, Current_val);
//		Current_Rms = C_cal_rms(Current_val);//实际控制可以注释，防止sqrt占用时间

//		Voltage_Rms_2 = V_cal_rms(Voltage_val_2);//-0.11f-0.5f;//0.3f
//		Voltage_Rms_Filtered = MovingAverage_Update(&voltage_filter,Voltage_Rms_2);		

//		sogi_pll_update(&Grid_PLL, Voltage_val);//锁相环更新
//		sogi_pll_update(&Grid_PLL, Voltage_val_2);//锁相环更新		

//		//角度向索引值映射
//    {
//			float theta_out = Grid_PLL.theta + phase_offset_rad;
//			theta_out = fmodf(theta_out, 6.2831853f);
//			if (theta_out < 0.0f) theta_out += 6.2831853f;

//			k = (uint16_t)(theta_out * (400.0f / 6.2831853f));
//			if (k >= 400) k = 399;
//		}
		
		//************************************三相电流过流检测************************************
		if (wave_enable_tim8)
		{
			
			float i_abs_max = fmaxf(fabsf(measurements.iu),
			                         fmaxf(fabsf(measurements.iv),
			                               fabsf(measurements.iw)));

			if (i_abs_max > CURRENT_TRIP_A)
			{
				overcurrent_count++;
			}
			else
			{
				overcurrent_count = 0;
			}

			if (overcurrent_count >= OVERCURRENT_TRIP_COUNT)
			{
				fault_latched = 1;
				inverter_status.fault_latched = 1U;
			}
		
			// 故障状态处理：关闭SVPWM输出，故障指示灯闪烁
			if (fault_latched)
			{
				Inverter_Wave_Stop_TIM8();			
				count++;
				if (count >= 2000)
				{
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
					count = 0;
				}
				return;
			}
		}
		else// 停机状态：保持PWM关闭，故障指示灯同样周期闪烁
		{
			count++;
			if (count >= 2000)
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
				count = 0;
			}
			return;
		}

//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((U_u+60.0f)/120.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((U_w+60.0f)/120.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((Three_phase_I_u+6.0f)/12.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((Three_phase_I_v+6.0f)/12.0f)*4096));
		//************************************占空比更新************************************//		

		{
			DqControlInput control_input;
			DqControlOutput control_output;
			SvpwmDuty duty;
			float sin_theta;
			float cos_theta;

			theta += 2.0f * 3.1415926f * 50.0f / 20000.0f;
			if (theta >= 6.2831853f)
			{
				theta -= 6.2831853f;
			}
			sin_theta = sinf(theta);
			cos_theta = cosf(theta);

			if (stop_requested)
			{
				soft_start_ratio -= SOFT_START_STEP;
				if (soft_start_ratio <= 0.0f)
				{
					soft_start_ratio = 0.0f;
					stop_requested = 0;
					Inverter_Wave_Stop_TIM8();
					return;
				}
			}
			else if (soft_start_ratio < 1.0f)
			{
				soft_start_ratio += SOFT_START_STEP;
				if (soft_start_ratio >= 1.0f)
				{
					soft_start_ratio = 1.0f;
				}
			}

			control_input.vd_ref = inverter_config.vll_ref_rms
			                       * DQ_SQRT_TWO_THIRDS
			                       * soft_start_ratio;
			control_input.u_u = measurements.u_u;
			control_input.u_vw = measurements.u_vw;
			control_input.iu = measurements.iu;
			control_input.iv = measurements.iv;
			control_input.iw = measurements.iw;
			control_input.sin_theta = sin_theta;
			control_input.cos_theta = cos_theta;

			DqControl_Step(&dq_control, &control_input, &control_output);

//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((v_alpha_feedback+30.0f)/60.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((v_beta_feedback+30.0f)/60.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((dq_vd_feedback+30.0f)/60.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((dq_vq_feedback+30.0f)/60.0f)*4096));

			Svpwm_Calculate(control_output.ud,
			                 control_output.uq,
			                 sin_theta,
			                 cos_theta,
			                 inverter_config.vdc,
			                 &duty);

			inverter_status.vd_ref = control_input.vd_ref;
			inverter_status.vd = control_output.vd;
			inverter_status.vq = control_output.vq;
			inverter_status.id_ref = control_output.id_ref;
			inverter_status.id = control_output.id;
			inverter_status.iq_ref = control_output.iq_ref;
			inverter_status.iq = control_output.iq;
			inverter_status.ud = control_output.ud;
			inverter_status.uq = control_output.uq;
			inverter_status.current_ref_limited =
				control_output.current_ref_limited;
			inverter_status.voltage_limited =
				control_output.voltage_limited;

			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
			                          (uint32_t)(duty.duty_a * 8400.0f));
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
			                          (uint32_t)(duty.duty_b * 8400.0f));
			__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
			                          (uint32_t)(duty.duty_c * 8400.0f));
		}
			count++;

     if(count>=2000)
     {
       HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);
       count=0;
     }
  }

//		// *** 归一化 ***
//		float normalized_target = target_v / MAX_VOLTAGE;   // 注意：target要先用pid_set_target设置原始值
//		float normalized_current = Voltage_Rms_2 / MAX_VOLTAGE;		
//		pid_set_target(&PID_Voltage, normalized_target);		
//		float pid_out_normalized = pid_calculate_positional(&PID_Voltage, normalized_current);		
//		PID_OUT_V = pid_out_normalized * MAX_MODULATION;
}
	








