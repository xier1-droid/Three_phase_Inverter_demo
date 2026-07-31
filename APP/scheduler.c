#include "scheduler.h"

uint8_t task_num; // 任务总数量

typedef struct // 任务结构体定义
{
    void (*task_func)(void); // 任务函数指针
    uint32_t rate_ms;        // 任务执行周期(毫秒)
    uint32_t last_run;       // 上次执行时间戳
} task_t;

// 系统任务列表，定义所有需要调度的任务及其执行周期
static task_t scheduler_task[] =
    {
        {Key_task, 10, 0},
				{uart_proc,20	,	0},
				{Display_task,200,0},
				{led_task,500,0},
				{uart_test,500	,	1},
//				{adc_task,15	,	0},
			
};

void scheduler_init(void) // 初始化任务调度器 参数:无 返回:无
{
    task_num = sizeof(scheduler_task) / sizeof(task_t); // 计算任务总数
}

void scheduler_run(void) // 运行任务调度器主循环 参数:无 返回:无
{
    for (uint8_t i = 0; i < task_num; i++) // 遍历所有任务
    {
        uint32_t now_time = HAL_GetTick(); // 获取当前系统时间

        if (now_time >= scheduler_task[i].rate_ms + scheduler_task[i].last_run) // 检查任务是否到达执行时间
        {
            scheduler_task[i].last_run = now_time; // 更新任务上次执行时间
            scheduler_task[i].task_func();         // 执行任务函数
        }
    }
}

void led_task(void)
{
//	printf("%.2f,%.2f",dq_voltage_kp,dq_voltage_ki);
//	my_printf(&huart1,"count:%d",count);
//	printf("run\r\n");
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_9);
//	pid_set_params(&PID_Voltage, P_v, I_v, 0);
}

static void AdjustVoltageReference(int32_t step_centivolts)
{
	InverterStatus status;
	int32_t reference_centivolts;

	Inverter_GetStatus(&status);
	reference_centivolts =
		(int32_t)(status.config.vll_ref_rms * 100.0f + 0.5f);
	reference_centivolts += step_centivolts;
	if (reference_centivolts < 0)
	{
		reference_centivolts = 0;
	}
	else if (reference_centivolts > 3400)
	{
		reference_centivolts = 3400;
	}

	(void)Inverter_SetParameter(INVERTER_PARAMETER_VLL_REF_RMS,
	                            (float)reference_centivolts / 100.0f);
}

void Key_task(void)
{
	Key_State key;

	KEY_Scan();
	key = KEY_GetState();
	switch (key)
	{
		case KEY1_PRESS:
			if (wave_enable_tim8 != 0U)
			{
				Inverter_Stop();
			}
			else
			{
				Inverter_Start();
			}
			break;

		case KEY2_PRESS:
			(void)Inverter_SetFrequency(30.0f);
			break;

		case KEY3_PRESS:
			(void)Inverter_SetFrequency(60.0f);
			break;

		case KEY4_PRESS:
			AdjustVoltageReference(-1);
			break;

		case KEY5_PRESS:
			AdjustVoltageReference(1);
			break;

		default:
			break;
	}
}

void Display_task(void)
{
	OLED_DisplayStatus();
}

void uart_test(void)
{
//	my_printf(&huart1,"PID_Voltage.target:%.2f,Voltage_Rms_2:%.2f\r\n",PID_Voltage.target,Voltage_Rms_2);
//	my_printf(&huart1,"PID_Voltage.target:%.2f,Voltage_Rms_2:%.2f\r\n",PID_Voltage.target,Voltage_Rms_2);
//	my_printf(&huart1,"KEY_GetState:%d\r\n",KEY_GetState());
//	my_printf(&huart1,"u_total:%f\r\n",PR_Current.u_total);
//	  my_printf(&huart1,"{Voltage_Get}%f\n{Current_rms}%f\n",Voltage_Get,Current_Rms);
//		my_printf(&huart1,"{Voltage_val}%f\n{Current_val}%f\n",Voltage_val,Current_val);
}
