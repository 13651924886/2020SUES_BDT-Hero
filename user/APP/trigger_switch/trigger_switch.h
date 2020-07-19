#ifndef TRIGGER_SWITCH_H
#define TRIGGER_SWITCH_H

#include "exit_init.h"

typedef struct
{
	bool_t trigger_state;
	
}trigger_switch_t;

typedef enum
{
    PushWheel = 0,
    ShootWheel
} trigger_switch_list;

extern trigger_switch_t *get_switch_point( trigger_switch_list object );
extern void trigger_switch_init(void);
extern void trigger_switch_in_PushWheel_detected(void);
extern void trigger_switch_in_ShootWheel_detected(void);
extern void trigger_switch_StateDelete( trigger_switch_list object );

 
#endif


