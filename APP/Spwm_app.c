#include "Spwm_app.h"



const float sine_wave[400] = {
    // === 第1象限 (100个点，正向递增) ===
    33.0f, 99.0f, 165.0f, 231.0f, 297.0f, 362.0f, 428.0f, 494.0f, 559.0f, 624.0f,
    690.0f, 755.0f, 819.0f, 884.0f, 948.0f, 1013.0f, 1076.0f, 1140.0f, 1203.0f, 1266.0f,
    1329.0f, 1392.0f, 1454.0f, 1515.0f, 1577.0f, 1638.0f, 1698.0f, 1758.0f, 1818.0f, 1877.0f,
    1936.0f, 1994.0f, 2052.0f, 2110.0f, 2166.0f, 2223.0f, 2278.0f, 2333.0f, 2388.0f, 2442.0f,
    2495.0f, 2548.0f, 2600.0f, 2652.0f, 2703.0f, 2753.0f, 2802.0f, 2851.0f, 2899.0f, 2946.0f,
    2993.0f, 3039.0f, 3084.0f, 3129.0f, 3172.0f, 3215.0f, 3257.0f, 3298.0f, 3339.0f, 3378.0f,
    3417.0f, 3455.0f, 3492.0f, 3528.0f, 3564.0f, 3598.0f, 3632.0f, 3664.0f, 3696.0f, 3727.0f,
    3757.0f, 3786.0f, 3814.0f, 3841.0f, 3868.0f, 3893.0f, 3917.0f, 3940.0f, 3963.0f, 3984.0f,
    4005.0f, 4024.0f, 4042.0f, 4060.0f, 4076.0f, 4092.0f, 4106.0f, 4119.0f, 4132.0f, 4143.0f,
    4153.0f, 4163.0f, 4171.0f, 4178.0f, 4184.0f, 4190.0f, 4194.0f, 4197.0f, 4199.0f, 4200.0f,

    // === 第2象限 (100个点，正向递减) ===
    4200.0f, 4199.0f, 4197.0f, 4194.0f, 4190.0f, 4184.0f, 4178.0f, 4171.0f, 4163.0f, 4153.0f,
    4143.0f, 4132.0f, 4119.0f, 4106.0f, 4092.0f, 4076.0f, 4060.0f, 4042.0f, 4024.0f, 4005.0f,
    3984.0f, 3963.0f, 3940.0f, 3917.0f, 3893.0f, 3868.0f, 3841.0f, 3814.0f, 3786.0f, 3757.0f,
    3727.0f, 3696.0f, 3664.0f, 3632.0f, 3598.0f, 3564.0f, 3528.0f, 3492.0f, 3455.0f, 3417.0f,
    3378.0f, 3339.0f, 3298.0f, 3257.0f, 3215.0f, 3172.0f, 3129.0f, 3084.0f, 3039.0f, 2993.0f,
    2946.0f, 2899.0f, 2851.0f, 2802.0f, 2753.0f, 2703.0f, 2652.0f, 2600.0f, 2548.0f, 2495.0f,
    2442.0f, 2388.0f, 2333.0f, 2278.0f, 2223.0f, 2166.0f, 2110.0f, 2052.0f, 1994.0f, 1936.0f,
    1877.0f, 1818.0f, 1758.0f, 1698.0f, 1638.0f, 1577.0f, 1515.0f, 1454.0f, 1392.0f, 1329.0f,
    1266.0f, 1203.0f, 1140.0f, 1076.0f, 1013.0f, 948.0f, 884.0f, 819.0f, 755.0f, 690.0f,
    624.0f, 559.0f, 494.0f, 428.0f, 362.0f, 297.0f, 231.0f, 165.0f, 99.0f, 33.0f,

    // === 第3象限 (100个点，负向递增) ===
    -33.0f, -99.0f, -165.0f, -231.0f, -297.0f, -362.0f, -428.0f, -494.0f, -559.0f, -624.0f,
    -690.0f, -755.0f, -819.0f, -884.0f, -948.0f, -1013.0f, -1076.0f, -1140.0f, -1203.0f, -1266.0f,
    -1329.0f, -1392.0f, -1454.0f, -1515.0f, -1577.0f, -1638.0f, -1698.0f, -1758.0f, -1818.0f, -1877.0f,
    -1936.0f, -1994.0f, -2052.0f, -2110.0f, -2166.0f, -2223.0f, -2278.0f, -2333.0f, -2388.0f, -2442.0f,
    -2495.0f, -2548.0f, -2600.0f, -2652.0f, -2703.0f, -2753.0f, -2802.0f, -2851.0f, -2899.0f, -2946.0f,
    -2993.0f, -3039.0f, -3084.0f, -3129.0f, -3172.0f, -3215.0f, -3257.0f, -3298.0f, -3339.0f, -3378.0f,
    -3417.0f, -3455.0f, -3492.0f, -3528.0f, -3564.0f, -3598.0f, -3632.0f, -3664.0f, -3696.0f, -3727.0f,
    -3757.0f, -3786.0f, -3814.0f, -3841.0f, -3868.0f, -3893.0f, -3917.0f, -3940.0f, -3963.0f, -3984.0f,
    -4005.0f, -4024.0f, -4042.0f, -4060.0f, -4076.0f, -4092.0f, -4106.0f, -4119.0f, -4132.0f, -4143.0f,
    -4153.0f, -4163.0f, -4171.0f, -4178.0f, -4184.0f, -4190.0f, -4194.0f, -4197.0f, -4199.0f, -4200.0f,

    // === 第4象限 (100个点，负向递减) ===
    -4200.0f, -4199.0f, -4197.0f, -4194.0f, -4190.0f, -4184.0f, -4178.0f, -4171.0f, -4163.0f, -4153.0f,
    -4143.0f, -4132.0f, -4119.0f, -4106.0f, -4092.0f, -4076.0f, -4060.0f, -4042.0f, -4024.0f, -4005.0f,
    -3984.0f, -3963.0f, -3940.0f, -3917.0f, -3893.0f, -3868.0f, -3841.0f, -3814.0f, -3786.0f, -3757.0f,
    -3727.0f, -3696.0f, -3664.0f, -3632.0f, -3598.0f, -3564.0f, -3528.0f, -3492.0f, -3455.0f, -3417.0f,
    -3378.0f, -3339.0f, -3298.0f, -3257.0f, -3215.0f, -3172.0f, -3129.0f, -3084.0f, -3039.0f, -2993.0f,
    -2946.0f, -2899.0f, -2851.0f, -2802.0f, -2753.0f, -2703.0f, -2652.0f, -2600.0f, -2548.0f, -2495.0f,
    -2442.0f, -2388.0f, -2333.0f, -2278.0f, -2223.0f, -2166.0f, -2110.0f, -2052.0f, -1994.0f, -1936.0f,
    -1877.0f, -1818.0f, -1758.0f, -1698.0f, -1638.0f, -1577.0f, -1515.0f, -1454.0f, -1392.0f, -1329.0f,
    -1266.0f, -1203.0f, -1140.0f, -1076.0f, -1013.0f, -948.0f, -884.0f, -819.0f, -755.0f, -690.0f,
    -624.0f, -559.0f, -494.0f, -428.0f, -362.0f, -297.0f, -231.0f, -165.0f, -99.0f, -33.0f

};

float Mid_Value = 4200.0f;   

volatile uint16_t i = 0;//主机索引
volatile uint16_t k = 0;//从机索引
uint16_t count;                

//原始采样值
float Voltage_Get=0;
float Current_Get=0;
//有效值
float Voltage_Rms=0;
float Current_Rms=0;
//瞬时值
float Voltage_val=0;
float Current_val=0;	
//原始采样值0-3.3V
float ADC_v_val = 0;
float ADC_c_val = 0;

//原始采样值
float Voltage_Get_2=0;
float Current_Get_2=0;
//有效值
float Voltage_Rms_2=0;
float Current_Rms_2=0;
//瞬时值
float Voltage_val_2=0;
float Current_val_2=0;	
//原始采样值0-3.3V
float ADC_v_val_2 = 0;
float ADC_c_val_2 = 0;

//线电压
float U_uv,U_vw,U_uw;
//相电压
float U_u,U_v,U_w;
//相电流
float Three_phase_I_u,Three_phase_I_w,Three_phase_I_v;



#define RMS_WINDOW 400
float current_square_sum = 0.0f;
float current_buffer[RMS_WINDOW] = {0};  
uint16_t current_idx = 0;

/**
 * @brief 计算电流有效值(RMS)
 * @param x 输入的瞬时电流值
 * @return 返回电流有效值
 * @details 使用滑动窗口算法计算RMS值，窗口大小为400个采样点
 */
float C_cal_rms(float x)
{
    float square = x * x;                    // 计算瞬时电流的平方
    
    // 减去最老的一个平方值（把最早的数据“踢出去”）
    current_square_sum -= current_buffer[current_idx];
    
    // 存入新的平方值
    current_buffer[current_idx] = square;
    current_square_sum += square;
    
    // 指针往前走一格，满了就从头开始（环形）
    current_idx = (current_idx + 1) % RMS_WINDOW;
    
    // 计算当前窗口的均方值，再开方得到RMS
    float mean_square = current_square_sum / RMS_WINDOW;
    return sqrtf(mean_square);
}

// 全局变量或结构体里定义
float voltage_square_sum = 0.0f;
float voltage_buffer[RMS_WINDOW] = {0};  // 用来存最近400个电压的平方值
uint16_t voltage_idx = 0;

/**
 * @brief 计算电压有效值(RMS)
 * @param x 输入的瞬时电压值
 * @return 返回电压有效值
 * @details 使用滑动窗口算法计算RMS值，窗口大小为400个采样点
 */
float V_cal_rms(float x)
{
    float square = x * x;
    
    // 先减去最老的一个平方值
    voltage_square_sum -= voltage_buffer[voltage_idx];
    
    // 存入新的平方值
    voltage_buffer[voltage_idx] = square;
    voltage_square_sum += square;
    
    // 移动指针
    voltage_idx = (voltage_idx + 1) % RMS_WINDOW;
    
    // 计算RMS
    float mean_square = voltage_square_sum / RMS_WINDOW;
    return sqrtf(mean_square);
}

/*
float Voltage_Rms=0;
float Current_Rms=0;
float Voltage_Get=0;
float Current_Get=0;
*/

extern uint32_t adc_val_buffer[];
extern uint32_t adc_val_buffer_2[];
//有效值平均滤波器
MovingAverageFilter_t voltage_filter;
MovingAverageFilter_t current_filter;
float Voltage_Rms_Filtered = 0;
float Current_Rms_Filtered = 0;
//瞬时值低通滤波器
LowPassFilter_t LPF_Current;
//滤除50HZ得到偏移量 变量
LowPassFilter_t Voltage_Offset_Filter;
LowPassFilter_t Current_Offset_Filter;
LowPassFilter_t Voltage_Offset_Filter_2;
LowPassFilter_t Current_Offset_Filter_2;
//PID调节器
PID_T PID_Voltage;
float PID_OUT_V = 0.0f;
PID_T PID_Current;
float PID_OUT_C = 0.0f;	
//PR调节器
PR_T PR_Current;          
float PR_OUT_C = 0.0f;
//
PI_TypeDef PI_C;

//SOGI-PLL环
SOGI_PLL_T Grid_PLL;

float sine_norm,Iref_inst;

float phase_offset_rad = 0.0f;//1.570796f; //手动相位偏差调节

#define MAX_VOLTAGE     45.0f     // 根据你实际最大target修改，比如30.0f也行
#define MAX_MODULATION  1.0f      // PID_OUT的最大合理值，通常1.0就够

// —— 电流环软启动 ——
static float soft_start_ratio = 0.0f;      // 0.0 ~ 1.0，当前软启动系数
#define SOFT_START_CYCLES   50.0f   // 软启动持续的基波周期数（50Hz下 20周期≈0.4s）
#define SOFT_START_STEP     (1.0f / (SOFT_START_CYCLES * 400.0f))  // 每次TIM8中断（每周期400点）的增量

volatile uint8_t wave_enable_tim8 = 0;   // 0=未发波（上电默认），1=已发波
#define PHASE_SHIFT_B  133
#define PHASE_SHIFT_C  267

// ==== 三相开环 SVPWM（第一轮） ====
#define DQ_CONTROL_TS_S               0.00005f
#define DQ_SQRT_TWO_THIRDS            0.816496581f
#define DQ_INV_SQRT_THREE             0.577350269f
#define DQ_VDC_NOMINAL_V              60.0f
#define DQ_VECTOR_LIMIT_V             31.1769145f
#define DQ_PI_CORRECTION_LIMIT_V      3.0f
#define DQ_VOLTAGE_KP_MAX             2.0f
#define DQ_VOLTAGE_KI_MAX             500.0f
#define DQ_LINE_VOLTAGE_REF_MAX_V     32.0f

typedef struct
{
    float integral;
} DqVoltagePi;

static DqVoltagePi dq_vd_pi = {0.0f};
static DqVoltagePi dq_vq_pi = {0.0f};
static volatile float dq_voltage_kp = 0.0f;
static volatile float dq_voltage_ki = 0.0f;
static volatile float dq_line_voltage_ref_rms = 32.0f;
static volatile float dq_vd_reference = 0.0f;
static volatile float dq_vd_feedback = 0.0f;
static volatile float dq_vq_feedback = 0.0f;
static volatile float dq_ud_command = 0.0f;
static volatile float dq_uq_command = 0.0f;

static float theta = 0.0f;          // 角度发生器状态

// ===== 设备状态 + 启停控制 + 参数读写 =====
static volatile uint8_t fault_latched = 0;
static volatile uint8_t stop_requested = 0;

#define CURRENT_TRIP_A          4.0f
#define OVERCURRENT_TRIP_COUNT  3
static uint8_t overcurrent_count = 0;

static void DqVoltagePi_Reset(void)
{
    dq_vd_pi.integral = 0.0f;
    dq_vq_pi.integral = 0.0f;
    dq_vd_reference = 0.0f;
    dq_ud_command = 0.0f;
    dq_uq_command = 0.0f;
}

static float DqVoltagePi_Update(DqVoltagePi *pi, float error)
{
    float kp = dq_voltage_kp;
    float ki = dq_voltage_ki;
    float integral_candidate = pi->integral + ki * DQ_CONTROL_TS_S * error;
    float output = kp * error + integral_candidate;

    if (output > DQ_PI_CORRECTION_LIMIT_V)
    {
        if (error < 0.0f)
        {
            pi->integral = integral_candidate;
        }
        return DQ_PI_CORRECTION_LIMIT_V;
    }
    if (output < -DQ_PI_CORRECTION_LIMIT_V)
    {
        if (error > 0.0f)
        {
            pi->integral = integral_candidate;
        }
        return -DQ_PI_CORRECTION_LIMIT_V;
    }

    pi->integral = integral_candidate;
    return output;
}

void Inverter_SetVoltageKp(float kp)
{
    if ((kp >= 0.0f) && (kp <= DQ_VOLTAGE_KP_MAX))
    {
        dq_voltage_kp = kp;
    }
}

void Inverter_SetVoltageKi(float ki)
{
    if ((ki >= 0.0f) && (ki <= DQ_VOLTAGE_KI_MAX))
    {
        dq_voltage_ki = ki;
    }
}

void Inverter_SetLineVoltageRef(float vll_rms)
{
    if ((vll_rms >= 0.0f) && (vll_rms <= DQ_LINE_VOLTAGE_REF_MAX_V))
    {
        dq_line_voltage_ref_rms = vll_rms;
    }
}

float Inverter_GetVoltageKp(void)
{
    return dq_voltage_kp;
}

float Inverter_GetVoltageKi(void)
{
    return dq_voltage_ki;
}

float Inverter_GetLineVoltageRef(void)
{
    return dq_line_voltage_ref_rms;
}

void Inverter_GetVoltageStatus(InverterVoltageStatus *status)
{
    if (status == NULL)
    {
        return;
    }

    status->vll_ref_rms = dq_line_voltage_ref_rms;
    status->vd_ref = dq_vd_reference;
    status->vd = dq_vd_feedback;
    status->vq = dq_vq_feedback;
    status->ud_cmd = dq_ud_command;
    status->uq_cmd = dq_uq_command;
    status->kp = dq_voltage_kp;
    status->ki = dq_voltage_ki;
}

void Inverter_Wave_Start_TIM8(void)
{
    DqVoltagePi_Reset();
    soft_start_ratio = 0.0f;   // 从0开始软启动斜坡

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 4200);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 4200);
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 4200);
	
	    wave_enable_tim8 = 1;
}

void Inverter_Wave_Stop_TIM8(void)
{
    DqVoltagePi_Reset();
    wave_enable_tim8 = 0;

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
        DqVoltagePi_Reset();
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
					
		//************************************有效值核算************************************//
		Voltage_Get = adc_val_buffer[1];
		Current_Get = adc_val_buffer[0];
				
		//????????????(alpha???1,????????????,??????????)
		float v_offset = LowPass_Update(&Voltage_Offset_Filter, Voltage_Get);
		float c_offset = LowPass_Update(&Current_Offset_Filter, Current_Get);
		
		//????+???0-3.3V
		ADC_v_val = (Voltage_Get-v_offset) * 3.3f/4096.0f;
		ADC_c_val = (Current_Get-c_offset) * 3.3f/4096.0f;
				
		Current_val = ADC_c_val * 4.629f; //4.51325581f;//2000.0f)/(4.0f*100.0f);//0.5-4.19;0.8~1.0-4.49;2.02A-4.629
		Voltage_val = ((ADC_v_val * ((39.0f / 2.0f) * 1000))/(3.922f*150.0f))*0.92342f;//32.0V
				
//		Current_val = (Current_Get * 3.3f/4096.0f) * 4.96f - 7.956f;      // 电流信号标定(A)//4.96
//		Voltage_val = (Voltage_Get * 3.3f/4096.0f) * 33.125f - 53.66f;  // 电压信号标定(V)
		
//		Voltage_Rms = V_cal_rms(Voltage_val);
//		Voltage_Rms_Filtered = MovingAverage_Update(&voltage_filter,Voltage_Rms);

//		Current_val = LowPass_Update(&LPF_Current, Current_val);
//		Current_Rms = C_cal_rms(Current_val);//实际控制可以注释，防止sqrt占用时间
		
		Voltage_Get_2 = adc_val_buffer_2[1];
		Current_Get_2 = adc_val_buffer_2[0];		
		
		float v_offset_2 = LowPass_Update(&Voltage_Offset_Filter_2, Voltage_Get_2);
		float c_offset_2 = LowPass_Update(&Current_Offset_Filter_2, Current_Get_2);	
		
		ADC_v_val_2 = (Voltage_Get_2-v_offset_2) * 3.3f/4096.0f;
		ADC_c_val_2 = (Current_Get_2-c_offset_2) * 3.3f/4096.0f;		
		
		Current_val_2 = ADC_c_val_2 * 4.57f;//2000.0f)/(4.0f*100.0f);//0.37~0.5→4.121;0.8~1.0→4.39;2.07A-4.57
		Voltage_val_2 = ((ADC_v_val_2 * ((39.0f / 2.0f) * 1000))/(4.0f*150.0f))*0.9324f;//32.0f				
		
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
		
		U_uv = Voltage_val_2;
		U_vw = Voltage_val;
		U_uw = U_uv+U_vw;
		
		Three_phase_I_u = Current_val_2;
		Three_phase_I_w = Current_val;
		// 基尔霍夫电流定律（无中性线电流）求 v 相电流
		Three_phase_I_v = -(Three_phase_I_u + Three_phase_I_w);
				
		// 三相平衡假设下由线电压反解相电压
		U_u = (2.0f * U_uv + U_vw) / 3.0f;
		U_v = (U_vw - U_uv) / 3.0f;
		U_w = -(U_uv + 2.0f * U_vw) / 3.0f;
		
		//************************************三相电流过流检测************************************
		if (wave_enable_tim8)
		{
			
			float i_abs_max = fmaxf(fabsf(Three_phase_I_u),fmaxf(fabsf(Three_phase_I_v), fabsf(Three_phase_I_w)));

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

		// 角度发生器：50Hz @ 20kHz
		theta += 2.0f * 3.1415926f * 50.0f / 20000.0f;
		if (theta >= 6.2831853f)
		{
			theta -= 6.2831853f;
		}

		float sin_theta = sinf(theta);
		float cos_theta = cosf(theta);

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

		/* U_uv = U_u - U_v and U_vw = U_v - U_w. */
		float v_alpha_feedback = U_u;
		float v_beta_feedback = (U_v - U_w) * DQ_INV_SQRT_THREE;
		dq_vd_feedback = v_alpha_feedback * cos_theta
		                 + v_beta_feedback * sin_theta;
		dq_vq_feedback = -v_alpha_feedback * sin_theta
		                 + v_beta_feedback * cos_theta;

		/* vref is line-to-line RMS voltage; vd_ref is phase peak voltage. */
		dq_vd_reference = dq_line_voltage_ref_rms
		                  * DQ_SQRT_TWO_THIRDS
		                  * soft_start_ratio;

		float ud_correction = DqVoltagePi_Update(
			&dq_vd_pi, dq_vd_reference - dq_vd_feedback);
		float uq_correction = DqVoltagePi_Update(
			&dq_vq_pi, -dq_vq_feedback);

		dq_ud_command = dq_vd_reference + ud_correction;
		dq_uq_command = uq_correction;

		float vector_magnitude_sq = dq_ud_command * dq_ud_command
		                            + dq_uq_command * dq_uq_command;
		float vector_limit_sq = DQ_VECTOR_LIMIT_V * DQ_VECTOR_LIMIT_V;
		if (vector_magnitude_sq > vector_limit_sq)
		{
			float vector_scale = DQ_VECTOR_LIMIT_V / sqrtf(vector_magnitude_sq);
			dq_ud_command *= vector_scale;
			dq_uq_command *= vector_scale;
		}

		float u_alpha = dq_ud_command * cos_theta
		                - dq_uq_command * sin_theta;
		float u_beta = dq_ud_command * sin_theta
		               + dq_uq_command * cos_theta;

		float ua = u_alpha;
		float ub = -0.5f * u_alpha + 0.8660254f * u_beta;
		float uc = -0.5f * u_alpha - 0.8660254f * u_beta;

		float u_max = fmaxf(ua, fmaxf(ub, uc));
		float u_min = fminf(ua, fminf(ub, uc));
		float u_zero = -0.5f * (u_max + u_min);

		float ua_svpwm = ua + u_zero;
		float ub_svpwm = ub + u_zero;
		float uc_svpwm = uc + u_zero;

		float duty_a = 0.5f + ua_svpwm / DQ_VDC_NOMINAL_V;
		float duty_b = 0.5f + ub_svpwm / DQ_VDC_NOMINAL_V;
		float duty_c = 0.5f + uc_svpwm / DQ_VDC_NOMINAL_V;

		duty_a = fminf(fmaxf(duty_a, 0.05f), 0.95f);
		duty_b = fminf(fmaxf(duty_b, 0.05f), 0.95f);
		duty_c = fminf(fmaxf(duty_c, 0.05f), 0.95f);

		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,
		                          (uint32_t)(duty_a * 8400.0f));
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,
		                          (uint32_t)(duty_b * 8400.0f));
		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,
		                          (uint32_t)(duty_c * 8400.0f));
			count++;

     if(count>=2000)
     {
       HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);
       count=0;
     }
  }
	if(htim->Instance == TIM13)  
  {
		
//		// *** 归一化 ***
//		float normalized_target = target_v / MAX_VOLTAGE;   // 注意：target要先用pid_set_target设置原始值
//		float normalized_current = Voltage_Rms_2 / MAX_VOLTAGE;		
//		pid_set_target(&PID_Voltage, normalized_target);		
//		float pid_out_normalized = pid_calculate_positional(&PID_Voltage, normalized_current);		
//		PID_OUT_V = pid_out_normalized * MAX_MODULATION;
		
	}
	
}
	








