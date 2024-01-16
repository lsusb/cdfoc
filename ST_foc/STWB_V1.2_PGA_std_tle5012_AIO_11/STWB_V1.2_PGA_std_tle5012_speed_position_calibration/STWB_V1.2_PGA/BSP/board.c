#include "board.h"

#include "bsp_usart3.h"
#include "bsp_key.h"

#include "bsp_tle5012.h"
#include "bsp_spi.h"
#include "bsp_can.h"
#include "bsp_at24cxx.h"

#include "filter.h"
#include "control.h" 



#include "parameters_conversion.h"


u8 Init_OK;


#define TICK_PER_SECOND 2000
#define TICK_US	(1000000/TICK_PER_SECOND)
volatile uint32_t sysTickUptime=0;

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
    while (GetSysTime_us() - now < us);
}

void DelayMs(uint32_t ms)
{
    while (ms--)
        DelayUs(1000);
}



void TDT_Board_ALL_Init(void)
{
	
		SPI3_Init();
	
		Tle5012_Init();
	
		Usart3_Init();
			
		CAN1_Configuration();

		At24c02_Init();
	
		KEY_Init();

		
		MotorControl.info.eeprom_s = !AT24CXX_Check();

		LPF2pSetCutoffFreq_1(3000,1000);
		LPF2pSetCutoffFreq_2(3000,1000);
	
//		MotorControl.parameter.pos_compensation = -17000;
//		MotorControl.info.mag_alignment = 1;
	
		MotorControl.info.can_id = 1;
		MotorControl.info.motor_driver_name = 10;   //10   DY-TestVersion
		MotorControl.parameter.control_mode = 2;
		MotorControl.parameter.max_angle = 500;
	
		MotorControl.parameter.speed_parameter.kp = 40;
		MotorControl.parameter.speed_parameter.ki = 4000;
		MotorControl.parameter.speed_parameter.li = 10;  //6
		
		MotorControl.parameter.pos_parameter.kp = 0.2 *100;
		MotorControl.parameter.pos_parameter.ki = 0;
		MotorControl.parameter.pos_parameter.li = 10;
		
		MotorControl.parameter.max_speed = 300;
		MotorControl.parameter.feed_back_freq	= 1000;
	
		MotorControl.info.version_number = 2;
		
		if(MotorControl.info.eeprom_s)
				EEPROM_ReadParameter();
		
//		MotorControl.info.mag_alignment = 0;

		Init_OK = 1;
}
