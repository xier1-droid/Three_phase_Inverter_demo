#include "Spwm_app.h"
#include "Inverter_sampling.h"
#include "Svpwm.h"

#define DQ_SQRT_TWO_THIRDS 0.816496581f
#define CURRENT_TRIP_A 5.0f
#define OVERCURRENT_TRIP_COUNT 3U
#define PWM_PERIOD_COUNTS 8400.0f
#define PWM_IDLE_COMPARE 4200U
#define INDICATOR_TOGGLE_TICKS 2000U
#define FUNDAMENTAL_FREQUENCY_HZ 50.0f
#define CONTROL_FREQUENCY_HZ 20000.0f
#define CONTROL_SAMPLES_PER_CYCLE 400U
#define CONTROL_CYCLE_AVERAGE_SCALE (1.0f / 400.0f)
#define VCOMP_LIMIT_V 0.5f
#define VCOMP_SLOPE_LIMIT_V_PER_A 0.5f
#define VCOMP_FILTER_GAIN 0.0015696f
#define VCOMP_EFFECTIVE_REF_MAX_V 32.5f
#define VCOMP_DEFAULT_OFFSET_V (-0.1569f)
#define VCOMP_DEFAULT_SLOPE_V_PER_A -0.1053f
#if DQ_CONTROL_MODE == DQ_VOLTAGE_LOOP
#define VCOMP_DEFAULT_ENABLED 1U
#else
#define VCOMP_DEFAULT_ENABLED 0U
#endif

static float soft_start_ratio = 0.0f;      // 0.0 ~ 1.0����ǰ������ϵ��
#define SOFT_START_CYCLES   50.0f   // �����������Ļ�����������50Hz�� 20���ڡ�0.4s��
#define SOFT_START_STEP     (1.0f / (SOFT_START_CYCLES * 400.0f))  // ÿ��TIM8�жϣ�ÿ����400�㣩������

volatile uint8_t wave_enable_tim8 = 0;   // 0=δ�������ϵ�Ĭ�ϣ���1=�ѷ���
// ===== �豸״̬ + ��ͣ���� + ������д =====
static volatile uint8_t fault_latched = 0;
static volatile uint8_t stop_requested = 0;
static uint8_t overcurrent_count = 0;
static uint8_t tim8_pwm_channels_started = 0;
static uint16_t count = 0U;
static float theta = 0.0f;
static InverterConfig inverter_config;
InverterStatus inverter_status;
static DqControl dq_control;
InverterMeasurements measurements;
static float cycle_vd_sum;
static float cycle_vq_sum;
static float cycle_u_uv_square_sum;
static float cycle_u_vw_square_sum;
static float cycle_u_wu_square_sum;
static float cycle_iu_square_sum;
static float cycle_iv_square_sum;
static float cycle_iw_square_sum;
static uint16_t cycle_sample_count;
static float voltage_compensation_target_v;
static float voltage_compensation_applied_v;
static float effective_vll_ref_rms;

static void Inverter_ResetCycleDiagnostics(void)
{
    cycle_vd_sum = 0.0f;
    cycle_vq_sum = 0.0f;
    cycle_u_uv_square_sum = 0.0f;
    cycle_u_vw_square_sum = 0.0f;
    cycle_u_wu_square_sum = 0.0f;
    cycle_iu_square_sum = 0.0f;
    cycle_iv_square_sum = 0.0f;
    cycle_iw_square_sum = 0.0f;
    cycle_sample_count = 0U;
    inverter_status.vd_cycle_average = 0.0f;
    inverter_status.vq_cycle_average = 0.0f;
    inverter_status.u_uv_cycle_mean_square = 0.0f;
    inverter_status.u_vw_cycle_mean_square = 0.0f;
    inverter_status.u_wu_cycle_mean_square = 0.0f;
    inverter_status.iu_cycle_mean_square = 0.0f;
    inverter_status.iv_cycle_mean_square = 0.0f;
    inverter_status.iw_cycle_mean_square = 0.0f;
    inverter_status.load_current_rms = 0.0f;
    inverter_status.cycle_diagnostic_valid = 0U;
}

static void Inverter_UpdateCycleDiagnostics(const DqControlOutput *output)
{
    float u_wu = -(measurements.u_uv + measurements.u_vw);
    float load_current_mean_square;

    cycle_vd_sum += output->vd;
    cycle_vq_sum += output->vq;
    cycle_u_uv_square_sum += measurements.u_uv * measurements.u_uv;
    cycle_u_vw_square_sum += measurements.u_vw * measurements.u_vw;
    cycle_u_wu_square_sum += u_wu * u_wu;
    cycle_iu_square_sum += measurements.iu * measurements.iu;
    cycle_iv_square_sum += measurements.iv * measurements.iv;
    cycle_iw_square_sum += measurements.iw * measurements.iw;
    cycle_sample_count++;

    if (cycle_sample_count < CONTROL_SAMPLES_PER_CYCLE)
    {
        return;
    }

    inverter_status.vd_cycle_average =
        cycle_vd_sum * CONTROL_CYCLE_AVERAGE_SCALE;
    inverter_status.vq_cycle_average =
        cycle_vq_sum * CONTROL_CYCLE_AVERAGE_SCALE;
    inverter_status.u_uv_cycle_mean_square =
        cycle_u_uv_square_sum * CONTROL_CYCLE_AVERAGE_SCALE;
    inverter_status.u_vw_cycle_mean_square =
        cycle_u_vw_square_sum * CONTROL_CYCLE_AVERAGE_SCALE;
    inverter_status.u_wu_cycle_mean_square =
        cycle_u_wu_square_sum * CONTROL_CYCLE_AVERAGE_SCALE;
    inverter_status.iu_cycle_mean_square =
        cycle_iu_square_sum * CONTROL_CYCLE_AVERAGE_SCALE;
    inverter_status.iv_cycle_mean_square =
        cycle_iv_square_sum * CONTROL_CYCLE_AVERAGE_SCALE;
    inverter_status.iw_cycle_mean_square =
        cycle_iw_square_sum * CONTROL_CYCLE_AVERAGE_SCALE;
    load_current_mean_square =
        (inverter_status.iu_cycle_mean_square
         + inverter_status.iv_cycle_mean_square
         + inverter_status.iw_cycle_mean_square) / 3.0f;
    inverter_status.load_current_rms = sqrtf(load_current_mean_square);
    inverter_status.cycle_diagnostic_valid = 1U;

    cycle_vd_sum = 0.0f;
    cycle_vq_sum = 0.0f;
    cycle_u_uv_square_sum = 0.0f;
    cycle_u_vw_square_sum = 0.0f;
    cycle_u_wu_square_sum = 0.0f;
    cycle_iu_square_sum = 0.0f;
    cycle_iv_square_sum = 0.0f;
    cycle_iw_square_sum = 0.0f;
    cycle_sample_count = 0U;
}

static float Inverter_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

static void Inverter_ResetVoltageCompensation(void)
{
    voltage_compensation_target_v = 0.0f;
    voltage_compensation_applied_v = 0.0f;
    effective_vll_ref_rms = inverter_config.vll_ref_rms;
    inverter_status.voltage_compensation_target_v = 0.0f;
    inverter_status.voltage_compensation_applied_v = 0.0f;
    inverter_status.effective_vll_ref_rms = effective_vll_ref_rms;
}

static void Inverter_UpdateVoltageCompensation(void)
{
#if DQ_CONTROL_MODE == DQ_VOLTAGE_LOOP
    if (inverter_config.voltage_compensation_enabled == 0U)
    {
        voltage_compensation_target_v = 0.0f;
        voltage_compensation_applied_v = 0.0f;
    }
    else
    {
        if (inverter_status.cycle_diagnostic_valid != 0U)
        {
            voltage_compensation_target_v =
                inverter_config.voltage_compensation_offset_v
                + inverter_config.voltage_compensation_slope_v_per_a
                  * inverter_status.load_current_rms;
            voltage_compensation_target_v =
                Inverter_Clamp(voltage_compensation_target_v,
                               -VCOMP_LIMIT_V,
                               VCOMP_LIMIT_V);
        }
        else
        {
            voltage_compensation_target_v = 0.0f;
        }

        voltage_compensation_applied_v +=
            VCOMP_FILTER_GAIN
            * (voltage_compensation_target_v
               - voltage_compensation_applied_v);
    }
#else
    voltage_compensation_target_v = 0.0f;
    voltage_compensation_applied_v = 0.0f;
#endif

    effective_vll_ref_rms =
        Inverter_Clamp(inverter_config.vll_ref_rms
                       + voltage_compensation_applied_v,
                       0.0f,
                       VCOMP_EFFECTIVE_REF_MAX_V);
    inverter_status.voltage_compensation_target_v =
        voltage_compensation_target_v;
    inverter_status.voltage_compensation_applied_v =
        voltage_compensation_applied_v;
    inverter_status.effective_vll_ref_rms = effective_vll_ref_rms;
}

static void Inverter_ResetControlState(void)
{
    DqControl_Reset(&dq_control);
    Inverter_ResetCycleDiagnostics();
    Inverter_ResetVoltageCompensation();
    inverter_status.vd_ref = 0.0f;
    inverter_status.vd = 0.0f;
    inverter_status.vq = 0.0f;
    inverter_status.voltage_d_integral = 0.0f;
    inverter_status.voltage_q_integral = 0.0f;
    inverter_status.ud = 0.0f;
    inverter_status.uq = 0.0f;
    inverter_status.voltage_limited = 0U;
}

void Inverter_Init(void)
{
    memset(&inverter_status, 0, sizeof(inverter_status));
    memset(&measurements, 0, sizeof(measurements));

    inverter_config.vll_ref_rms = 32.00f;
    inverter_config.voltage_kp = 0.028f;
    inverter_config.voltage_ki = 0.008f;
    inverter_config.voltage_compensation_offset_v = VCOMP_DEFAULT_OFFSET_V;
    inverter_config.voltage_compensation_slope_v_per_a =
        VCOMP_DEFAULT_SLOPE_V_PER_A;
    inverter_config.voltage_compensation_enabled = VCOMP_DEFAULT_ENABLED;

    DqControl_Init(&dq_control, &inverter_config);
    Inverter_ResetVoltageCompensation();
    inverter_status.mode = DQ_CONTROL_MODE;
    inverter_status.config = inverter_config;
}

bool Inverter_SetParameter(InverterParameter parameter, float value)
{
    InverterConfig new_config = inverter_config;
    uint32_t primask;

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

        case INVERTER_PARAMETER_VCOMP_ENABLE:
            if (!((value == 0.0f) || (value == 1.0f)))
            {
                return false;
            }
            new_config.voltage_compensation_enabled = (uint8_t)value;
            break;

        case INVERTER_PARAMETER_VCOMP_OFFSET:
            if (!((value >= -VCOMP_LIMIT_V) &&
                  (value <= VCOMP_LIMIT_V)))
            {
                return false;
            }
            new_config.voltage_compensation_offset_v = value;
            break;

        case INVERTER_PARAMETER_VCOMP_SLOPE:
            if (!((value >= -VCOMP_SLOPE_LIMIT_V_PER_A) &&
                  (value <= VCOMP_SLOPE_LIMIT_V_PER_A)))
            {
                return false;
            }
            new_config.voltage_compensation_slope_v_per_a = value;
            break;

        default:
            return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    inverter_config = new_config;
    DqControl_SetConfig(&dq_control, &inverter_config);
    inverter_status.config = inverter_config;
    if (inverter_config.voltage_compensation_enabled == 0U)
    {
        Inverter_ResetVoltageCompensation();
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
    soft_start_ratio = 0.0f;   // ��0��ʼ������б��

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
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_IDLE_COMPARE);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PWM_IDLE_COMPARE);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, PWM_IDLE_COMPARE);
	
	    wave_enable_tim8 = 1;
        inverter_status.running = 1U;
}

void Inverter_Wave_Stop_TIM8(void)
{
    Inverter_ResetControlState();
    wave_enable_tim8 = 0;
    inverter_status.running = 0U;

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, PWM_IDLE_COMPARE);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, PWM_IDLE_COMPARE);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, PWM_IDLE_COMPARE);

    /* Disable PWM outputs, but keep TIM8 counter and update IRQ running. */
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    soft_start_ratio = 0.0f;   // ����������б��
}

// ����� ״̬���ƽӿ�(�ļ���Uart_cmp.c) ��������ͣ����ֱ�Ӳ���Ӳ��
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
      stop_requested = 1;   // ��ֹͣ�����־���ȴ�б�½�Ƶͣ��
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

static void Inverter_UpdateIndicator(void)
{
    count++;
    if (count >= INDICATOR_TOGGLE_TICKS)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
        count = 0U;
    }
}

static bool Inverter_CheckOvercurrent(void)
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
        overcurrent_count = 0U;
    }

    if (overcurrent_count < OVERCURRENT_TRIP_COUNT)
    {
        return false;
    }

    fault_latched = 1U;
    inverter_status.fault_latched = 1U;
    Inverter_Wave_Stop_TIM8();
    return true;
}

static bool Inverter_UpdateSoftStart(void)
{
    if (stop_requested != 0U)
    {
        soft_start_ratio -= SOFT_START_STEP;
        if (soft_start_ratio <= 0.0f)
        {
            soft_start_ratio = 0.0f;
            stop_requested = 0U;
            Inverter_Wave_Stop_TIM8();
            return false;
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

    return true;
}

static void Inverter_RunVoltageControl(void)
{
    DqControlInput control_input;
    DqControlOutput control_output;
    SvpwmDuty duty;
    float sin_theta;
    float cos_theta;

    theta += 2.0f * 3.1415926f * FUNDAMENTAL_FREQUENCY_HZ
             / CONTROL_FREQUENCY_HZ;
    if (theta >= 6.2831853f)
    {
        theta -= 6.2831853f;
    }
    sin_theta = sinf(theta);
    cos_theta = cosf(theta);
    Inverter_UpdateVoltageCompensation();

    control_input.vd_ref = effective_vll_ref_rms
                           * DQ_SQRT_TWO_THIRDS
                           * soft_start_ratio;
    control_input.vdc = measurements.vdc;
    control_input.u_u = measurements.u_u;
    control_input.u_vw = measurements.u_vw;
    control_input.sin_theta = sin_theta;
    control_input.cos_theta = cos_theta;

    DqControl_Step(&dq_control, &control_input, &control_output);
    Svpwm_Calculate(control_output.ud,
                     control_output.uq,
                     sin_theta,
                     cos_theta,
                     measurements.vdc,
                     &duty);

    inverter_status.vd_ref = control_input.vd_ref;
    inverter_status.vd = control_output.vd;
    inverter_status.vq = control_output.vq;
    inverter_status.voltage_d_integral = control_output.voltage_d_integral;
    inverter_status.voltage_q_integral = control_output.voltage_q_integral;
    inverter_status.ud = control_output.ud;
    inverter_status.uq = control_output.uq;
    inverter_status.voltage_limited = control_output.voltage_limited;
    Inverter_UpdateCycleDiagnostics(&control_output);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
                          (uint32_t)(duty.duty_a * PWM_PERIOD_COUNTS));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
                          (uint32_t)(duty.duty_b * PWM_PERIOD_COUNTS));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
                          (uint32_t)(duty.duty_c * PWM_PERIOD_COUNTS));
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
		inverter_status.vdc = measurements.vdc;
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
		                 (uint32_t)(((measurements.u_vw + 50.0f) / 100.0f)
		                            * 4096.0f));
		
		//************************************��������������************************************
		if (wave_enable_tim8 == 0U)
		{
			Inverter_UpdateIndicator();
			return;
		}
		if (Inverter_CheckOvercurrent())
		{
			Inverter_UpdateIndicator();
			return;
		}

//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((U_u+60.0f)/120.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((U_w+60.0f)/120.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((Three_phase_I_u+6.0f)/12.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((Three_phase_I_v+6.0f)/12.0f)*4096));
		//************************************ռ�ձȸ���************************************//		

		if (!Inverter_UpdateSoftStart())
		{
			return;
		}

//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((v_alpha_feedback+30.0f)/60.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((v_beta_feedback+30.0f)/60.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t)(((dq_vd_feedback+30.0f)/60.0f)*4096));
//		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t)(((dq_vq_feedback+30.0f)/60.0f)*4096));

		Inverter_RunVoltageControl();
		Inverter_UpdateIndicator();
  }

//		// *** ��һ�� ***
//		float normalized_target = target_v / MAX_VOLTAGE;   // ע�⣺targetҪ����pid_set_target����ԭʼֵ
//		float normalized_current = Voltage_Rms_2 / MAX_VOLTAGE;		
//		pid_set_target(&PID_Voltage, normalized_target);		
//		float pid_out_normalized = pid_calculate_positional(&PID_Voltage, normalized_current);		
//		PID_OUT_V = pid_out_normalized * MAX_MODULATION;
}
	








