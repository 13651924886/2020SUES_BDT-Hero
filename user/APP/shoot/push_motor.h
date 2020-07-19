#ifndef _PUSH_MOTOR_H_
#define _PUSH_MOTOR_H_

#include "stm32f4xx.h"

#define PUSH_CW_SPEED 1550		//1520 - 1920
#define PUSH_CCW_SPEED 1400		//1080 - 1480


void Push_Wheel_Stop(void);
void Push_Wheel_CCW(void);
void Push_Wheel_CW(void);



#endif

