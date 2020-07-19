#include "delay.h"
#include "led.h"
#include "stm32f4xx.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;
u32 systime_ms;
volatile uint32_t sysTickUptime = 0;


void delay_init(uint32_t TICK_RATE_HZ)
{
    uint32_t reload = 0;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);    // 180/8=22.5mhz
    fac_us = SystemCoreClock / 8000000;  										// 180mhz/8mhz =22.5mhz
    fac_ms = SystemCoreClock / 8000;

    if (TICK_RATE_HZ == 0)
    {
        TICK_RATE_HZ = 1000;
    }

    reload = SystemCoreClock / TICK_RATE_HZ / 8;            //  180mhz/8/1000 = 22500
    reload--;

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->LOAD = reload;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void delay_us(uint16_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void delay_ms(uint16_t nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void sys_time()
{
	systime_ms++;
}

uint32_t SysTick_GetTick(void)
{
	return systime_ms;
}

uint32_t GetSysTime_us ( void )
{
    register uint32_t ms;
    u32 value;
	do
	{
    ms = sysTickUptime;
    value = ms * TICK_US + ( SysTick->LOAD - SysTick->VAL ) * TICK_US / SysTick->LOAD;
	}
	while(ms != sysTickUptime);
	return value;
}


void SysTick_Handler(void)
{
 
	sysTickUptime++;
	sys_time();
	//sys_time();   //systime_ms变量每1ms累加1
	//LED_1ms_DRV();
	flow_led_Combo1();
}


