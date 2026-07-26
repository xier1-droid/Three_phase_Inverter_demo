#include "Adc_cmp.h"

//// --- 宏定义和外部变量 ---
//#define BUFFER_SIZE 800        // DMA 缓冲区大小 (总点数)

//extern DMA_HandleTypeDef hdma_adc1; // 假设这是 ADC1 对应的 DMA 句柄
//extern ADC_HandleTypeDef hadc1;    // ADC1 句柄
//extern UART_HandleTypeDef huart1; // 用于 my_printf 的 UART 句柄

//// --- 全局变量 ---
//volatile float Current_rms;//PA0
//volatile float Voltage_rms;//PA1
__IO uint32_t adc_val_buffer[2]; // DMA 目标缓冲区 (存储原始 ADC1 数据)
__IO uint32_t adc_val_buffer_2[2]; // DMA 目标缓冲区 (存储原始 ADC2 数据)

//__IO uint8_t AdcConvEnd = 0;             // ADC 转换完成标志 (一个块完成)

// --- 初始化函数 (在 main 或外设初始化后调用) ---
void adc_tim_dma_init(void)
{
	// 启动 ADC 的 DMA 传输，请求 BUFFER_SIZE 个数据点
	// 注意：这里假设 hadc1 已经配置为合适的触发模式 (定时器或软件)
	//       且 DMA 配置为 Normal 模式
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val_buffer, 2);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_val_buffer_2, 2);

	// 显式禁用 DMA 半传输中断 (如果不需要处理半满事件)
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(&hdma_adc2, DMA_IT_HT);

//    // 注意：如果使用定时器触发，需要在此处或之前启动定时器
//    HAL_TIM_Base_Start(&htim8); // 替换 htimX 为实际定时器句柄
}

//// --- ADC 转换完成回调函数 (由 DMA TC 中断触发) ---
//// 当 DMA 完成整个缓冲区的传输 (Normal 模式下传输 BUFFER_SIZE 个点) 时触发
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    // 检查是否是由我们关心的 ADC (hadc1) 触发的
//    if (hadc->Instance == ADC1) // 或 if(hadc == &hadc1)
//    {
//        HAL_ADC_Stop_DMA(hadc);

//        // 设置转换完成标志，通知后台任务数据已准备好
//        AdcConvEnd = 1;
//    }
//}

//// --- 后台处理任务 (在主循环或低优先级任务中调用) ---
//void adc_task(void)
//{
//    float sum_square_V = 0.0f;
//    float sum_square_I = 0.0f;
//    float Current_temp = 0.0f;
//    float Voltage_temp = 0.0f;
//	
//    // 检查转换完成标志
//    if (AdcConvEnd)
//    {
//			uint16_t i ; 
////			my_printf(&huart1, "BUFFER_SIZE:%d\n",sizeof(adc_val_buffer)/sizeof(uint32_t));
//        // 处理数据: 从原始 ADC 缓冲区提取数据到 dac_val_buffer
//        // 示例逻辑：提取扫描转换中第二个通道的数据
//        for( i = 0; i < BUFFER_SIZE / 2; i++)
//        {
//            // 假设 adc_val_buffer[0] 是通道1, adc_val_buffer[1] 是通道2, ...
//					//转换为0~3.3V
//					Current_temp = adc_val_buffer[i * 2] * 3.3/4096.0f;
//					Voltage_temp = adc_val_buffer[i * 2 + 1] * 3.3/4096.0f;
//					//换算为实际电压
//					Current_temp = (Current_temp-1.65f)/4.0f/100.0f * 2000.0f;
//					Voltage_temp = (Voltage_temp-1.65f)/4.0f/150.0f * (39.0f / 2.0f);
//					//平方累加
//					sum_square_V += Voltage_temp * Voltage_temp;
//					sum_square_I += Current_temp * Current_temp;
//        }
////				my_printf(&huart1,"i: %hu\r\n",i);
//				
//				Voltage_rms = sqrtf( (float)sum_square_V / 400.0f );
//				Current_rms = sqrtf( (float)sum_square_I / 400.0f );
//				
//				
//				
////        // 打印处理后的数据
////        for(uint16_t i = 0; i < BUFFER_SIZE / 2; i++)
////        {
////					
////					my_printf(&huart1, "{Current_buffer}:%d\n{Voltage_buffer}:%d\n|%d", (int)Current_buffer[i],(int)Voltage_buffer[i],i);
////        }
//        // 清理处理后的缓冲区 (可选)
////				memset(Current_buffer, 0, sizeof(uint32_t) * (BUFFER_SIZE / 2));
////        memset(Voltage_buffer, 0, sizeof(uint32_t) * (BUFFER_SIZE / 2));

//        // 清除转换完成标志，准备下一次采集
//        AdcConvEnd = 0;

//        // 重新启动 ADC 的 DMA 传输，采集下一个数据块
//        // 注意: 需要确保 ADC 状态适合重启 (例如没有错误)
//        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val_buffer, BUFFER_SIZE);
//        // 再次禁用半传输中断 (如果 Start_DMA 会重新启用它)
//        __HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
//    }
//}
