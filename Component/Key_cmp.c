#include "Key_cmp.h"

// 全局变量：保存按键状态（volatile确保编译器不优化）
volatile Key_State g_key_state = KEY_NONE;
volatile uint8_t g_key_flag = 0;
/**
 * @brief  外部中断回调函数（HAL库统一入口，需重写）
 * @param  GPIO_Pin: 触发中断的引脚
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//    delay_us(10000);//!!!!!
    
    // 确认引脚确实为高电平（上升沿触发，按键按下后电平变化）
    if(HAL_GPIO_ReadPin(GPIOE, GPIO_Pin) == GPIO_PIN_RESET)
    {
			__disable_irq();
        switch(GPIO_Pin)
        {
            case GPIO_PIN_1:
                g_key_state = KEY1_PRESS;
                break;
            case GPIO_PIN_2:
                g_key_state = KEY2_PRESS;
                break;
            case GPIO_PIN_3:
                g_key_state = KEY3_PRESS;
                break;
            case GPIO_PIN_4:
                g_key_state = KEY4_PRESS;
                break;
            case GPIO_PIN_5:
                g_key_state = KEY5_PRESS;
                break;
            case GPIO_PIN_6:
                g_key_state = KEY6_PRESS;
                break;
            default:
                g_key_state = KEY_NONE;
                break;
        }
				__enable_irq();
    }
    
    // 清除中断标志（HAL库自动清除，可选）
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);
}

/**
 * @brief  获取按键状态（供主函数调用）
 * @retval 按键状态枚举
 */
Key_State KEY_GetState(void)
{
    Key_State temp = g_key_state;
    // 原子操作读取并清空状态
    __disable_irq();
    if(g_key_flag == 1)
    {
        temp = g_key_state;
        g_key_state = KEY_NONE;
        g_key_flag = 0; // 仅读取后清空标记
    }
    __enable_irq();
    return temp;
}
