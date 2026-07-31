#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "mydefine.h" // 全局定义头文件

void scheduler_init(void); // 初始化任务调度器 参数:无 返回:无
void scheduler_run(void);  // 运行任务调度器主循环 参数:无 返回:无

void led_task(void);
void Key_task(void);
void Display_task(void);
void uart_test(void);
	
#endif
