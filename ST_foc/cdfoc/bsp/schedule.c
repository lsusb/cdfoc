#include "schedule.h"
#include "control.h" 


#include "board.h"
#include "mc_tasks.h"
#include "main.h"

void TDT_Loop_1000Hz(void)//1ms执行一次
{
	float loop_time_1000hz = Get_Cycle_T(1);    	  /*获取5ms准确时间*/ 
	UNUSED(loop_time_1000hz);

}

void TDT_Loop_500Hz(void)	//2ms执行一次
{
    float loop_time_500hz;
    loop_time_500hz = Get_Cycle_T(2);    	  /*获取5ms准确时间*/
	UNUSED(loop_time_500hz);
	
}

void TDT_Loop_200Hz(void)	//5ms执行一次
{
    float loop_time_200hz;
    loop_time_200hz = Get_Cycle_T(3);    	  /*获取5ms准确时间*/
	UNUSED(loop_time_200hz);
}

void TDT_Loop_100Hz(void)	//10ms执行一次
{
	float loop_time_100hz = Get_Cycle_T(4);
	UNUSED(loop_time_100hz);
}

void TDT_Loop_50Hz(void)	//20ms执行一次
{
    float loop_time_50hz = Get_Cycle_T(5);
	UNUSED(loop_time_50hz);


}

uint8_t drv_fault = 1;
void TDT_Loop_20Hz(void)	//50ms执行一次
{
    static u8 timer_50ms = 0;//记录50ms次数
    float loop_time_20hz = Get_Cycle_T(5);
	UNUSED(loop_time_20hz);
	
	drv_fault = HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port,DRV_FAULT_Pin);
	
    if(++timer_50ms > 20)
    {
        timer_50ms = 0;
		HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
    }
}

volatile float Cycle_T[GET_TIME_NUM][3];

float Get_Cycle_T(u8 item)
{
    Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
    Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f; //本次的时间
    Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//间隔的时间（周期）
    return Cycle_T[item][NEW];
}

void TDT_Cycle_Time_Init(void)
{
    u8 i;
    for(i=0; i<GET_TIME_NUM; i++)
    {
        Get_Cycle_T(i);
    }
}


void TDT_SYSTICK_IRQHandler(void)
{
		static schedule infantrySchedule;
		sysTickUptime++;
		infantrySchedule.cnt_1ms++;
		infantrySchedule.cnt_2ms++;
		infantrySchedule.cnt_5ms++;
		infantrySchedule.cnt_10ms++;
		infantrySchedule.cnt_20ms++;
		infantrySchedule.cnt_50ms++;
	
		if(Init_OK)
		TDT_Loop(&infantrySchedule);
}


void TDT_Loop(schedule* robotSchdule)
{
    if(robotSchdule->cnt_1ms >= 1)
    {
        TDT_Loop_1000Hz();
        robotSchdule->cnt_1ms = 0;
    }
    if(robotSchdule->cnt_2ms >= 2)
    {
        TDT_Loop_500Hz();
        robotSchdule->cnt_2ms = 0;
    }
    if(robotSchdule->cnt_5ms >= 5)
    {
        TDT_Loop_200Hz();
        robotSchdule->cnt_5ms = 0;
    }
    if(robotSchdule->cnt_10ms >= 10)
    {
        TDT_Loop_100Hz();
        robotSchdule->cnt_10ms = 0;
    }
    if(robotSchdule->cnt_20ms >= 20)
    {
        TDT_Loop_50Hz();
        robotSchdule->cnt_20ms = 0;
    }
    if(robotSchdule->cnt_50ms >= 50)
    {
        TDT_Loop_20Hz();
        robotSchdule->cnt_50ms = 0;
    }
}


