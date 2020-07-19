#include "push_motor.h"


void Push_Wheel_Stop()
{
	TIM_SetCompare1(TIM5,1520);
}


void Push_Wheel_CW()
{
	TIM_SetCompare1(TIM5,PUSH_CW_SPEED);
}

void Push_Wheel_CCW()
{
	TIM_SetCompare1(TIM5,PUSH_CCW_SPEED);
}