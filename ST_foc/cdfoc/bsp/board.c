#include "board.h"

#include "filter.h"
#include "control.h"
#include "hqfoc_app.h"

#include "parameters_conversion.h"

u8 Init_OK;


volatile uint32_t sysTickUptime = 0;

uint32_t GetSysTime_us(void)
{
    register uint32_t ms;
    u32 value;
    ms = sysTickUptime;
    value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
    return value;
}

void DelayUs(uint32_t us)
{
    uint32_t now = GetSysTime_us();
    while (GetSysTime_us() - now < us)
        ;
}

void DelayMs(uint32_t ms)
{
    while (ms--)
        DelayUs(1000);
}

void HQ_Board_ALL_Init(void)
{
    drv_init();

    tle_5012_init();
    
    Init_OK = 1;
}
