#ifndef UART_APP_H
#define UART_APP_H

#include "mydefine.h"

void Uart_init(void);
int my_printf(UART_HandleTypeDef *huart, const char *format, ...);
void uart_proc(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif

