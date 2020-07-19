#include "trigger_switch.h"

trigger_switch_t trigger_switch_in_PushWheel;	//拨轮处的限位开关
trigger_switch_t trigger_switch_in_ShootWheel;	//拨叉处的限位开关

void trigger_switch_init(void)
{
	trigger_switch_in_PushWheel.trigger_state = 0;
	trigger_switch_in_ShootWheel.trigger_state = 0;
}

trigger_switch_t *get_switch_point( trigger_switch_list object )
{
	if(object == PushWheel)	return &trigger_switch_in_PushWheel;
	if(object == ShootWheel) return &trigger_switch_in_ShootWheel;
}

void trigger_switch_in_PushWheel_detected(void)
{
		trigger_switch_in_PushWheel.trigger_state = 1;
}

void trigger_switch_in_ShootWheel_detected(void)
{
	trigger_switch_in_ShootWheel.trigger_state = 1;
}

void trigger_switch_StateDelete( trigger_switch_list object )
{
	if(object == PushWheel)	trigger_switch_in_PushWheel.trigger_state = 0;
	if(object == ShootWheel) trigger_switch_in_ShootWheel.trigger_state = 0;
}
