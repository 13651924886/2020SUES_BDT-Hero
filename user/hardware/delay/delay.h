#ifndef DELAY_H
#define DELAY_H
#include "main.h"
#include "stm32f4xx.h"

#define TICK_PER_SECOND	1000
#define TICK_US	(1000000/TICK_PER_SECOND)

extern void delay_init(uint32_t TICK_RATE_HZ);
extern void delay_us(uint16_t nus);
extern void delay_ms(uint16_t nms);
extern void sys_time(void);
extern uint32_t SysTick_GetTick(void);
extern uint32_t GetSysTime_us(void);


#endif

