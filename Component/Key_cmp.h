#ifndef __KEY_CMP_H
#define __KEY_CMP_H	

#include "mydefine.h"

typedef enum {
    KEY_NONE = 0,
    KEY1_PRESS,
    KEY2_PRESS,
    KEY3_PRESS,
    KEY4_PRESS,
    KEY5_PRESS,
    KEY6_PRESS,
    KEY6_LONG_PRESS
} Key_State; 

void KEY_Init(void);
void KEY_Scan(void);
Key_State KEY_GetState(void);

#endif
