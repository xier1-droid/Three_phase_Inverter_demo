#ifndef __KEY_CMP_H
#define __KEY_CMP_H	

#include "mydefine.h"

// 按键状态枚举（可选，方便管理）
typedef enum {
    KEY_NONE = 0,
    KEY1_PRESS,  // PE1
    KEY2_PRESS,  // PE2
    KEY3_PRESS,  // PE3
    KEY4_PRESS,  // PE4
    KEY5_PRESS,  // PE5
    KEY6_PRESS   // PE6
} Key_State; 

// 函数声明
void KEY_EXTI_Callback(uint16_t GPIO_Pin);
Key_State KEY_GetState(void);

#endif
