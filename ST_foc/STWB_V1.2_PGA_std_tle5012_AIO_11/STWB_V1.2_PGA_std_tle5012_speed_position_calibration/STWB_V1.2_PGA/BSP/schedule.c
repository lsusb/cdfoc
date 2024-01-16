#include "schedule.h"
#include "control.h" 
#include "bsp_at24cxx.h"
#include "bsp_key.h"
#include "bsp_can.h"

#include "board.h"

#include "led.h"

#include "bsp_tle5012.h"


#include "mc_tasks.h"

void TDT_Loop_1000Hz(void)//1msִ��һ��
{
		float loop_time_1000hz;
		loop_time_1000hz = Get_Cycle_T(1);    	  /*��ȡ5ms׼ȷʱ��*/ 
//		Read_TLE5012_STD_STATE_REG();
		set_value_planer();
		if(MotorControl.info.motor_init)
			MotorControlApp(loop_time_1000hz);
		
		UsartSendFeed_back();
		CanSendFeed_back();

}

void TDT_Loop_500Hz(void)	//2msִ��һ��
{
    float loop_time_500hz;
    loop_time_500hz = Get_Cycle_T(2);    	  /*��ȡ5ms׼ȷʱ��*/
//		MotorControlApp(loop_time_500hz);
}

void TDT_Loop_200Hz(void)	//5msִ��һ��
{
    float loop_time_200hz;
    static uint16_t timer_5ms = 0,fst_state=0;//��¼5ms��
    loop_time_200hz = Get_Cycle_T(3);    	  /*��ȡ5ms׼ȷʱ��*/
}

void TDT_Loop_100Hz(void)	//10msִ��һ��
{
		float loop_time_100hz = Get_Cycle_T(4);
}

void TDT_Loop_50Hz(void)	//20msִ��һ��
{
//    float loop_time_50hz = Get_Cycle_T(5);

//    TDT_IWDG_Feed();


}

void TDT_Loop_20Hz(void)	//50msִ��һ��
{
    static u8 timer_50ms = 0;//��¼50ms����
    float loop_time_20hz = Get_Cycle_T(5);
		MotorControl.info.can_id = Get_keyValue();
		key_can_id = MotorControl.info.can_id;
    if(++timer_50ms > 10)
    {
        timer_50ms = 0;
				LED_GREEN_TOGGLE;
    }
}

volatile float Cycle_T[GET_TIME_NUM][3];

float Get_Cycle_T(u8 item)
{
    Cycle_T[item][OLD] = Cycle_T[item][NOW];	//��һ�ε�ʱ��
    Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f; //���ε�ʱ��
    Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//�����ʱ�䣨���ڣ�
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


