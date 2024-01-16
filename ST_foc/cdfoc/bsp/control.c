#include "control.h" 
#include "board.h"

#include "parameter.h"
#include "bsp_tle5012.h"
#include "bsp_can.h"
#include "filter.h"
#include "bsp_usart3.h"
#include "bsp_at24cxx.h"
#include "crc.h"


#include "mc_tasks.h"
#include "mc_math.h"
#include "mc_config.h"
#include "mc_type.h"

#include "arm_math.h"


extern PWMC_Handle_t * pwmcHandle[NBR_OF_MOTORS];
extern CircleLimitation_Handle_t *pCLM[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDIq[NBR_OF_MOTORS];
extern PID_Handle_t *pPIDId[NBR_OF_MOTORS];


extern volatile uint16_t TLE5012_reg;
extern volatile uint16_t TLE5012_reg_14;

extern volatile int16_t TLE5012_speed;

_mt_control MotorControl;

float t1,t2,time_t;
int16_t target_id = 0;
 
 
qd_t Vqd_temp;

 
void Foc_Control(void)
{
		static uint16_t TLE5012_reg_last = 0;
		static long TLE5012_cnt = 0;
//		t1 = GetSysTime_us();

		//��ǶȻ�ȡ//
//		SPI_DMA_WRITE_READ_BUF();
//		Read_TLE5012_STD();
		Read_TLE5012_STD();
	
//		MotorControl.parameter.pos_compensation = 17500;
//		MotorControl.parameter.pos_compensation = -13000;
	
		if(TLE5012_reg_last - TLE5012_reg_14 > 16384/2)
				TLE5012_cnt ++;
		if(TLE5012_reg_last - TLE5012_reg_14 < -16384/2)
				TLE5012_cnt --;
		MotorControl.tle5012.pos = TLE5012_reg_14 + TLE5012_cnt * 16384;
		TLE5012_reg_last = TLE5012_reg_14;
		
		MotorControl.tle5012.pos_filter = SlideAverageFilter(MotorControl.tle5012.pos);
		
		if(MotorControl.info.motor_init == 1)
		{
				if(MotorControl.info.mag_alignment == 0)
				{
//						MotorControl.foc.hElAngle += 2;
//						MotorControl.foc.Vqd.d = 10000;
//					
//						if(++alignment_time > 1000*30)
//						{
//								if(TLE5012_reg == 0)
//								{
//										MotorControl.info.mag_alignment = 1;
//										MotorControl.parameter.pos_compensation = MotorControl.foc.hElAngle;
//										MotorControl.pos.set_pos = MotorControl.tle5012.pos;
//										MotorControl.foc.Vqd.d = 0;
//										MotorControl.info.eeprom_write = 1;
//								}
//						}
					
						Auto_mag_alignment();
	
				}
				
				MotorControl.foc.hElAngle = -(TLE5012_reg * 2 *11 + MotorControl.parameter.pos_compensation);
				
//				if(MotorControl.info.mag_alignment)
//					MotorControl.foc.hElAngle = -(TLE5012_reg * 2 *11 + MotorControl.parameter.pos_compensation);
		
		}

		
		//�������ȡ//
		PWMC_GetPhaseCurrents(pwmcHandle[M1], &MotorControl.foc.Iab);
		
		
		//Clarke�任//
		MotorControl.foc.Ialphabeta = MCM_Clarke(MotorControl.foc.Iab);
	
		//Park�任//
		MotorControl.foc.Iqd = MCM_Park(MotorControl.foc.Ialphabeta, MotorControl.foc.hElAngle);
	
//		MotorControl.foc.Iqd_f.q = LPF2pApply_1(MotorControl.foc.Iqd.q);
//		MotorControl.foc.Iqd_f.d = LPF2pApply_2(MotorControl.foc.Iqd.d);
		

		
		//����������Vd��Vq// 
		if(MotorControl.info.motor_init && MotorControl.info.mag_alignment)
		{
//				Current_IqControl(&MotorControl.vel.output,&MotorControl.foc.Iqd.q,&MotorControl.foc.Vqd.q,0.0333f);
//				Current_IdControl(&target_id,&MotorControl.foc.Iqd.d,&MotorControl.foc.Vqd.d,0.0333f);
			
				MotorControl.foc.Vqd.q = PI_Controller(pPIDIq[M1],
									(int32_t)(MotorControl.vel.output) - MotorControl.foc.Iqd.q);

//				MotorControl.foc.Vqd.d = PI_Controller(pPIDId[M1],
//									(int32_t)(target_id) - MotorControl.foc.Iqd.d);
			
//				DelayUs(2);
				
//				Current_IqControl(&MotorControl.vel.output,&MotorControl.foc.Iqd.q,&Vqd_temp.q,0.0333f);
//				Current_IdControl(&target_id,&MotorControl.foc.Iqd.d,&Vqd_temp.d,0.0333f);
		}
//		MotorControl.foc.Vqd.q = 6000;
		
		//���������Ʊȶ�Vd��Vq�޷�//
		MotorControl.foc.Vqd = Circle_Limitation(pCLM[M1], MotorControl.foc.Vqd);

		//��Park�任//
		MotorControl.foc.Valphabeta = MCM_Rev_Park(MotorControl.foc.Vqd, MotorControl.foc.hElAngle);
	
		//����PWM//
		PWMC_SetPhaseVoltage(pwmcHandle[M1], MotorControl.foc.Valphabeta);
		
//		t2 = GetSysTime_us();
		
		
		
//		time_t = t2 - t1;
}






void Current_IdControl(int16_t* setValue, int16_t* fbValue, int16_t* result, float T)
{
    static pid currentId;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&currentId, pidCurrent_Id);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    currentId.setValue = *setValue;
    /* ����ֵ */
    currentId.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    currentId.error = currentId.setValue - currentId.feedbackValue;
    /* ���������� */
    currentId.pOut = currentId.kp * currentId.error;
    /* ƫ����л��� */
    currentId.integralError += currentId.error * T;
    /* ƫ��Ļ��ֽ������� */
    currentId.integralError = LIMIT(currentId.integralError, -currentId.integralErrorMax, currentId.integralErrorMax);
    /* ���������� */
    currentId.iOut = currentId.ki * currentId.integralError;
    /* �ܵ���� = ���������� */
    currentId.out = currentId.pOut + currentId.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(currentId.out, -32767, 32767);

}

 
 
void Current_IqControl(float* setValue, int16_t* fbValue, int16_t* result, float T)
{
    static pid currentIq;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&currentIq, pidCurrent_Iq);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    currentIq.setValue = *setValue;
    /* ����ֵ */
    currentIq.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    currentIq.error = currentIq.setValue - currentIq.feedbackValue;
    /* ���������� */
    currentIq.pOut = currentIq.kp * currentIq.error;
    /* ƫ����л��� */
    currentIq.integralError += currentIq.error * T;
    /* ƫ��Ļ��ֽ������� */
    currentIq.integralError = LIMIT(currentIq.integralError, -currentIq.integralErrorMax, currentIq.integralErrorMax);
    /* ���������� */
    currentIq.iOut = currentIq.ki * currentIq.integralError;
    /* �ܵ���� = ���������� */
    currentIq.out = currentIq.pOut + currentIq.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(currentIq.out, -32767, 32767);

}

Filter_t speed_filter;
int speed_set_debug = 0;
void MotorControlApp(float T)
{
		static u8 cal_pos = 0;
		static u8 control_mode_last = 0;
		static uint16_t init_delat = 0;
		static float pos_last,deta_pos;
		
		deta_pos = MotorControl.tle5012.pos_filter - pos_last;
		
		pos_last = MotorControl.tle5012.pos_filter;
	
		MotorControl.tle5012.speed = deta_pos;
		
		MotorControl.vel.speed_rpm = MotorControl.tle5012.speed * 3.662109f;
		
		MotorControl.vel.feedbk_vel = MotorControl.tle5012.speed;
		MotorControl.pos.feedbk_pos = MotorControl.tle5012.pos;
	


//		Position_Control(&MotorControl.pos.set_pos,&MotorControl.pos.feedbk_pos,&MotorControl.vel.output,T);
	
//		if(can1Feedback.cmd.demonstration_mode == 1)
//				MotorControl.pos.set_pos = can1Feedback.GM3510_sumpos*2;
//		else if(can1Feedback.cmd.demonstration_mode == 3)
//				MotorControl.pos.set_pos += can1Feedback.cmd.ch0/660.0f * 80;
//		else if(can1Feedback.cmd.demonstration_mode == 7)
//				MotorControl.pos.set_pos += 1;
//		else if(can1Feedback.cmd.demonstration_mode == 9)
//				MotorControl.pos.set_pos = 0;
//		else if(can1Feedback.cmd.demonstration_mode == 8)
//				MotorControl.pos.set_pos = 16384/2;
//		else if(can1Feedback.cmd.demonstration_mode == 4)
//				MotorControl.pos.set_pos = 16384/2;
//		else if(can1Feedback.cmd.demonstration_mode == 6)
//				MotorControl.pos.set_pos = 16384/4;
//		else if(can1Feedback.cmd.demonstration_mode == 5)
//				MotorControl.pos.set_pos = 0;
//		else
//				MotorControl.pos.set_pos = 0;



//		MotorControl.pos.set_pos += HardLIMIT((usart_RxMsg.set_pos - MotorControl.pos.set_pos),-3000,3000) *0.01f;
//		MotorControl.pos.set_pos = usart_RxMsg.set_pos;
//		MotorControl.pos.set_pos += 2;
		
		if(init_delat > 300)
		{
				if(MotorControl.parameter.control_mode != 0)
					Speed_Control(&MotorControl.vel.set_vel,&MotorControl.vel.feedbk_vel,&MotorControl.vel.output,T);
			
			
//				if(++cal_pos > 2)
//				{		
						if(MotorControl.parameter.control_mode == 2)
								Position_Control(&MotorControl.pos.set_pos,&MotorControl.pos.feedbk_pos,&MotorControl.vel.set_vel,T);
						cal_pos = 0;
//				}
				if(MotorControl.parameter.control_mode == 1 && control_mode_last != 1)
						MotorControl.vel.set_vel = 0;
				if(MotorControl.parameter.control_mode == 2 && control_mode_last !=2)
						MotorControl.pos.set_pos = MotorControl.pos.feedbk_pos;
				
				if(MotorControl.parameter.control_mode == 0 && control_mode_last != 0)
						MotorControl.vel.output = 0;
				control_mode_last = MotorControl.parameter.control_mode;				
		}
		else
		{
				init_delat++;
				MotorControl.pos.set_pos = MotorControl.pos.feedbk_pos;
		}
		
//		MotorControl.vel.output = 200;
}




void Speed_Control(float* setValue, float* fbValue, float* result, float T)
{
    static pid mc_speed;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&mc_speed, pidSpeed);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    mc_speed.setValue = *setValue;
		
    /* ����ֵ */
    mc_speed.feedbackValue = *fbValue;
		
    /* ƫ�� = �趨ֵ - ����ֵ */
    mc_speed.error = mc_speed.setValue - mc_speed.feedbackValue;
		MotorControl.controller.v_error = mc_speed.error;
		
    /* ���������� */
    mc_speed.pOut = mc_speed.kp * mc_speed.error;
		
    /* ƫ����л��� */

    mc_speed.integralError +=  mc_speed.ki * mc_speed.error * T;
		
    /* ƫ��Ļ��ֽ������� */
//    mc_speed.integralError = LIMIT(mc_speed.integralError, -mc_speed.integralErrorMax, mc_speed.integralErrorMax);
    mc_speed.integralError = HardLIMIT(mc_speed.integralError, -32767, 32767);
		MotorControl.controller.p_error = MotorControl.pos.set_pos - MotorControl.pos.feedbk_pos;
		
		/*λ��ģʽ���ֶ�̬�޷�*/
		if(MotorControl.parameter.control_mode == 2)    
				mc_speed.integralError = HardLIMIT(mc_speed.integralError, -fabs(MotorControl.controller.p_error) * mc_speed.integralErrorMax, fabs(MotorControl.controller.p_error) * mc_speed.integralErrorMax);
			
    /* ���������� */
		if(mc_speed.ki == 0)
				mc_speed.iOut = 0;
		else
				mc_speed.iOut = mc_speed.integralError;
    /* �ܵ���� = ���������� */
    mc_speed.out = mc_speed.pOut + mc_speed.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(mc_speed.out, -32767, 32767);
}


void Position_Control(float* setValue, float* fbValue, float* result, float T)
{
    static pid mc_position;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&mc_position, pidPos);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    mc_position.setValue = *setValue;
    /* ����ֵ */
    mc_position.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    mc_position.error = mc_position.setValue - mc_position.feedbackValue;
		MotorControl.controller.p_error = mc_position.error;
    /* ���������� */
    mc_position.pOut = mc_position.kp * mc_position.error;
    /* ƫ����л��� */
    mc_position.integralError += mc_position.error * T;
    /* ƫ��Ļ��ֽ������� */
    mc_position.integralError = LIMIT(mc_position.integralError, -mc_position.integralErrorMax, mc_position.integralErrorMax);
    /* ���������� */
    mc_position.iOut = mc_position.ki * mc_position.integralError;
    /* �ܵ���� = ���������� */
    mc_position.out = mc_position.pOut + mc_position.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(mc_position.out, -82, 82);
		
}


void GM3510_Speed_Control(float* setValue, float* fbValue, float* result, float T)
{
    static pid gm3510;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&gm3510, gm3510Speed);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    gm3510.setValue = *setValue;
    /* ����ֵ */
    gm3510.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    gm3510.error = gm3510.setValue - gm3510.feedbackValue;
    /* ���������� */
    gm3510.pOut = gm3510.kp * gm3510.error;
    /* ƫ����л��� */
    gm3510.integralError += gm3510.error * T;
    /* ƫ��Ļ��ֽ������� */
    gm3510.integralError = LIMIT(gm3510.integralError, -gm3510.integralErrorMax, gm3510.integralErrorMax);
    /* ���������� */
    gm3510.iOut = gm3510.ki * gm3510.integralError;
    /* �ܵ���� = ���������� */
    gm3510.out = gm3510.pOut + gm3510.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(gm3510.out, -29000, 29000);
}


void GM3510_Pos_Control(float* setValue, float* fbValue, float* result, float T)
{
    static pid gm3510;

    static u8 paraLoadFlag = 0;

    /* �������û�м��أ����ز���*/
    if(!paraLoadFlag)
    {
        Get_PIDparameters(&gm3510, gm3510Pos);
//        paraLoadFlag = 1;
    }

    /* �趨ֵ */
    gm3510.setValue = *setValue;
    /* ����ֵ */
    gm3510.feedbackValue = *fbValue;
    /* ƫ�� = �趨ֵ - ����ֵ */
    gm3510.error = gm3510.setValue - gm3510.feedbackValue;
    /* ���������� */
    gm3510.pOut = gm3510.kp * gm3510.error;
    /* ƫ����л��� */
    gm3510.integralError += gm3510.error * T;
    /* ƫ��Ļ��ֽ������� */
    gm3510.integralError = LIMIT(gm3510.integralError, -gm3510.integralErrorMax, gm3510.integralErrorMax);
    /* ���������� */
    gm3510.iOut = gm3510.ki * gm3510.integralError;
    /* �ܵ���� = ���������� */
    gm3510.out = gm3510.pOut + gm3510.iOut;
    /* �ܵ�������ܳ����������ֵ�ķ�Χ */
    *result = LIMIT(gm3510.out, -200000, 200000);
}


float HardLIMIT(float rawdata,float min,float max)
{
		if(rawdata > max)
				rawdata = max;
		if(rawdata < min)
				rawdata = min;
		return rawdata;
}



//void UsartSendFeed_back(void) 
//{
//    int i=0;
//	
//    uint8_t buffer[22];
//    buffer[0] = 0XA5;
//    buffer[1] = 0xAA;
//    buffer[2] = 0X00;
//    buffer[3] = 0x00;
//    buffer[4] = 0x00;
//	
//    buffer[5] = (u8)(int16_t)(TLE5012_reg);
//    buffer[6] =(u8)((int16_t)(TLE5012_reg)>>8);
//	
//    buffer[7] = (u8)(int16_t)((int16_t)(MotorControl.tle5012.speed *10));
//    buffer[8] =(u8)((int16_t)((int16_t)(MotorControl.tle5012.speed *10))>>8);
//	

////		

//    while(i<9)
//    {
//        Usart3_SendChar(buffer[i]);
//        i++;
//    }

//}

void UsartSendFeed_back(void) 
{
		if(usart_RxMsg.pc_cmd == 0)					//��������
		{
				usart3_tx_buffer[0] = 0XA5;
				usart3_tx_buffer[1] = 0xAA;
				usart3_tx_buffer[2] = 0X00;
				usart3_tx_buffer[3] = 0x00;
				usart3_tx_buffer[4] = 0x00;
				Append_CRC8_Check_Sum((unsigned char *)usart3_tx_buffer,5);
			
				usart3_tx_buffer[5] = (u8)(int16_t)(TLE5012_reg);
				usart3_tx_buffer[6] =(u8)((int16_t)(TLE5012_reg)>>8);
			
				usart3_tx_buffer[7] = (u8)(int16_t)((int16_t)(MotorControl.tle5012.speed *10));
				usart3_tx_buffer[8] =(u8)((int16_t)((int16_t)(MotorControl.tle5012.speed *10))>>8);
			
				usart3_tx_buffer[9] = (u8)(int16_t)(MotorControl.foc.Iqd.q);
				usart3_tx_buffer[10] =(u8)((int16_t)(MotorControl.foc.Iqd.q)>>8);
			
			
		}
		else if(usart_RxMsg.pc_cmd == 1)		//��λ���������
		{
				
				
				
				usart3_tx_buffer[0] = 0xAC;
				usart3_tx_buffer[1] = 0xBE;
				u16TOu8(2,(u8* )usart3_tx_buffer,MotorControl.info.version_number);									//�̼��汾��2-3
				Append_CRC8_Check_Sum((unsigned char *)usart3_tx_buffer,5);													//CRCУ�� 4
				
				usart3_tx_buffer[5]	= MotorControl.info.can_id;																			//can1_id
				usart3_tx_buffer[6] = MotorControl.info.motor_driver_name;													//�������
				
				usart3_tx_buffer[7] = MotorControl.parameter.control_mode;													//����ģʽ
				
				u16TOu8(8,(u8* )usart3_tx_buffer,MotorControl.parameter.max_angle);									//8-9���ת��
				
				u16TOu8(10,(u8* )usart3_tx_buffer,MotorControl.parameter.max_speed);								//10-11���ת��
				
				u16TOu8(12,(u8* )usart3_tx_buffer,MotorControl.parameter.feed_back_freq);						//12-13���ת��
				
				s16TOu8(14,(u8* )usart3_tx_buffer,MotorControl.parameter.speed_parameter.kp);	//14-15�ٶȻ�P
				
				s16TOu8(16,(u8* )usart3_tx_buffer,MotorControl.parameter.speed_parameter.ki);	//16-17�ٶȻ�I
				
				s16TOu8(18,(u8* )usart3_tx_buffer,MotorControl.parameter.pos_parameter.kp);		//18-19�ٶȻ�P
				
				s16TOu8(20,(u8* )usart3_tx_buffer,MotorControl.parameter.pos_parameter.ki);			//20-21�ٶȻ�I
				
				usart_RxMsg.pc_cmd = 0;
		}

		Usart3_send_DMA();

}

#define KEEP_SPEED_ZERO_TIME_MS 800

int16_t alignment_pos_num1 = 0;
int16_t alignment_pos_num2 = 0;

int16_t deta_t =0;

void Auto_mag_alignment(void)
{
		static float pos_last;
		static uint16_t time_cycle = 0;
		static u8 alignment_stage = 0;
		static u8 first_in = 0;
		static u8 alignment_redy = 0;
		static int16_t change_direction = 1;
		static uint16_t speed_up_time = 0;
		static uint16_t speed_zero_time = 0;
		
		static int32_t temp_sum = 0;
		
		if(++time_cycle > 20*10)
		{
				time_cycle = 0;
				if(first_in == 0)
				{
						MotorControl.foc.Vqd.d = 10000;
						first_in = 1;
				}
				if(alignment_redy == 0)
				{
						if(fabsf(MotorControl.tle5012.pos_filter - pos_last) < 600)
						{
								MotorControl.parameter.pos_compensation += 50;
								speed_up_time = 0;
						}
						else
						{
								speed_up_time++;
						}
						pos_last = MotorControl.tle5012.pos_filter;
						
						if(speed_up_time > KEEP_SPEED_ZERO_TIME_MS/10)
						{
								speed_up_time = 0;
								alignment_redy = 1;
						}
					
				}
				
				
				if(alignment_redy == 1)
				{
						if(fabsf(MotorControl.tle5012.pos_filter - pos_last) > 5)
						{
								MotorControl.parameter.pos_compensation += 50 * change_direction;
								speed_zero_time = 0;
						}
						else
						{
								speed_zero_time++;
						}
						pos_last = MotorControl.tle5012.pos_filter;
						
						if(speed_zero_time > KEEP_SPEED_ZERO_TIME_MS/10)
						{
								if(alignment_stage == 0)
								{
										alignment_pos_num1 = MotorControl.parameter.pos_compensation;

										MotorControl.parameter.pos_compensation += 9000 * change_direction;
										change_direction = -change_direction;
										alignment_stage = 1;
								}
								else if(alignment_stage == 1)
								{
										alignment_pos_num2 = MotorControl.parameter.pos_compensation;
										MotorControl.foc.Vqd.d = 0;
										temp_sum = (alignment_pos_num1 + alignment_pos_num2)/2;
										MotorControl.parameter.pos_compensation = temp_sum;
										alignment_redy = 2;
								}
								speed_zero_time = 0;

						}	
				}	
				
				if(alignment_redy == 2)
				{
						MotorControl.foc.Vqd.q = 10000;
						
						deta_t = MotorControl.tle5012.pos_filter - pos_last;
						pos_last = MotorControl.tle5012.pos_filter;
					
						if(__fabs(deta_t) > 600)
						{
								if(deta_t < 0)
								{
										MotorControl.parameter.pos_compensation += 32768;
										MotorControl.foc.Vqd.q = 0;
										MotorControl.info.mag_alignment = 1;
										MotorControl.info.eeprom_write = 1;
								}
								else
								{
										MotorControl.foc.Vqd.q = 0;
										MotorControl.info.mag_alignment = 1;
										MotorControl.info.eeprom_write = 1;
								}
						}
						
					
				}


		
		}
		
//								MotorControl.foc.Vqd.d = 0;
		//		MotorControl.info.mag_alignment = 1;
}


//float acc_limit = 0.001;
//float err_limit = 50;
//float out_put1,out_put2;
//float kp_smooth = 0.1,ki_smooth = 0.1,kd_smooth = 0;
//void set_value_smooth(float input_val,float *output_val)
//{
//		static float err,err_last,p_term,i_term,dif_err,d_term;
//		
//		static float speed,speed_limited,acc;
//	
//		err = input_val - out_put1;
//	
//		p_term = err * kp_smooth;
//	
//		err = HardLIMIT(err,-err_limit,err_limit);

//		i_term += err * ki_smooth;
//	
//		dif_err = err - err_last;
//		err_last = err;
//		
//		d_term = dif_err * kd_smooth;
//	
//		out_put1 = p_term + i_term + d_term;
//	
//	
//		speed = input_val - out_put2;
//	
//		acc = speed - speed_limited;
//	
//		acc = HardLIMIT(acc,-acc_limit,acc_limit);
//		
//		if(fabs(speed) < 2)
//		{
//				acc = 0;
//				speed_limited = 0;
//				out_put2 = input_val;
//		}
//		speed_limited += acc;
//		
//		out_put2 += speed_limited;
//		
//		if(input_val - *output_val > 0)
//		{
//				if(out_put1 < out_put2)
//					*output_val = out_put1;
//				else
//					*output_val = out_put2;
//		}
//		else
//				if(out_put1 < out_put2)
//					*output_val = out_put2;
//				else
//					*output_val = out_put1;
//}

int32_t myABS(int32_t value)
{
		if(value<0)
			return -value;
		else
			return value;
}

int32_t myLIMIT(int32_t value,int32_t min,int32_t max)
{
		if(value<min)
			return min;
		else if(value > max)
			return max;
		else
			return value;
}


_mt_setValue_planer planer;

void No_3_predictor(_mt_setValue_planer* P_planer)
{
		uint32_t pre_time;
		
		//t=as/jmax			t>0 
		pre_time = myABS(P_planer->acc_start[2]) / myABS(P_planer->Max_Jerk);			
		//ve = vs+(as-1/2*jmax*t)*t
		P_planer->vel_end[2] = P_planer->vel_start[2] + (P_planer->acc_start[2] - P_planer->Max_Jerk * pre_time / 2) * pre_time;		
		//s = (vs+(1/2*as-1/6*jmax*t)*t)*t
		P_planer->pos_deta[2] = (P_planer->vel_start[2] + (P_planer->acc_start[2] / 2 - P_planer->Max_Jerk * pre_time / 6) * pre_time) * pre_time;
	
}


int32_t get_No_3_MAX_Vs(_mt_setValue_planer* P_planer)
{
		uint32_t pre_time;
		int32_t		max_vs;
		//t=as/jmax			t>0 
		pre_time = myABS(P_planer->acc_start[2]) / myABS(P_planer->Max_Jerk);
		
		max_vs = P_planer->Max_Vel - (P_planer->acc_start[2] - P_planer->Max_Jerk * pre_time / 2) * pre_time;		
	
		return max_vs;
	
}


void No_5_predictor(_mt_setValue_planer* P_planer)
{
		uint32_t pre_time;
		
		uint32_t No6_exist = 0;
		uint32_t	acc_lim = 0;
		
		No6_exist = P_planer->Max_Acc * P_planer->Max_Acc / P_planer->Max_Jerk;
		
		//����׶�6���ڣ�acc�ܵ���Amax
		if(P_planer->vel_start[4] >= No6_exist)
		{
				pre_time = P_planer->Max_Acc / P_planer->Max_Jerk;					//��ֵ�����Ż�
			
				P_planer->acc_end[4] = -P_planer->Max_Jerk * pre_time;			//��ֵ�����Ż�
				
				P_planer->vel_end[4] = P_planer->vel_start[4] - P_planer->Max_Jerk * pre_time * pre_time / 2;
			
				P_planer->pos_deta[4] = (P_planer->vel_start[4] - P_planer->Max_Jerk * pre_time * pre_time / 6) * pre_time;
		}
		else
		{
				acc_lim = MCM_Sqrt(P_planer->vel_start[4] * P_planer->Max_Jerk);			//alim = sqrt(vs*jmax)
				
				pre_time = myABS(acc_lim) / P_planer->Max_Jerk;												//t=alim/jmax
			
				P_planer->acc_end[4] = -P_planer->Max_Jerk * pre_time;								//ae=-jmax*t
				
				P_planer->vel_end[4] = P_planer->vel_start[4] - P_planer->Max_Jerk * pre_time * pre_time / 2;				//ve=vs-1/2*jmax*t^2;
			
				P_planer->pos_deta[4] = (P_planer->vel_start[4] - P_planer->Max_Jerk * pre_time * pre_time / 6) * pre_time;		//detas=vs*t-1/6*jmax*t^3
			
		}
			
}

void No_7_predictor(_mt_setValue_planer* P_planer)
{
		uint32_t pre_time;
	
		//t=as/jmax			t>0 
		pre_time = myABS(P_planer->acc_start[6]) / myABS(P_planer->Max_Jerk);			
	
		//s = (vs+(1/2*as+1/6*jmax*t)*t)*t
		P_planer->pos_deta[6] = (P_planer->vel_start[6] + (P_planer->acc_start[6] / 2 + P_planer->Max_Jerk * pre_time / 6) * pre_time) * pre_time;
	
}

void No_6_predictor(_mt_setValue_planer* P_planer)
{
		uint32_t pre_time;
		
		pre_time = myABS((P_planer->vel_start[5] - P_planer->vel_end[5]) / P_planer->acc_start[5]);   //t=(vs-ve)/as
		
		P_planer->acc_end[5] = P_planer->acc_start[5];
		
		if(pre_time != 0)
				P_planer->pos_deta[5] = (P_planer->vel_start[5] + P_planer->acc_start[5] * pre_time / 2) * pre_time;
		else
				P_planer->pos_deta[5] = 0;
}



int64_t planer_t1;
int64_t planer_s1;
u8 planer_start = 0;

int32_t input_value = 0,output_value = 0;

int32_t planer_output = 0;

		 u8 step_now = 0;
		 int32_t acc = 0, vel = 0, pos = 0;
		 int32_t input_value_last = 0;

void set_value_planer(void)
{
//		static u8 step_now = 0;
//		static uint32_t acc = 0, vel = 0, pos = 0;
//		static uint32_t input_value_last = 0;
//		planer_t1 = (200 - 8) / 2;
//	
//		planer_s1 = (vel_s + (acc_s / 2 + j_max * planer_t1 / 6) * planer_t1 )*planer_t1;
	
		static u8 step_4_pre_computer = 0;				//��4�׶�Ԥ���㣬ֻ����1��;
		static u8 step_5_pre_computer = 0;				//��5�׶�Ԥ���㣬ֻ����1��;
	
		planer.Max_Jerk = 2;
		planer.Max_Acc = 200;
		planer.Max_Vel = 82000;
	
//		planer.acc_start[2] = 200;
//		planer.vel_start[2] = 10000;
//	
//		No_3_predictor(&planer);
	
		//����ı䣬��ʼ�滮�������һ�׶�
		if(input_value_last != input_value)
		{
				step_now = 1;
		}
		input_value_last = input_value;
		
		
		//��һ�׶�����
		if(step_now == 1)
		{
				acc += planer.Max_Jerk;
				vel += acc;
				pos += vel;
				
				planer.acc_start[2] = acc;
				planer.vel_start[2] = vel;
				No_3_predictor(&planer);
				
				planer.acc_start[4] = 0;
				planer.vel_start[4] = planer.vel_end[2];
				No_5_predictor(&planer);
			
				if(myABS(planer.vel_end[4]) > planer.Max_Acc * planer.Max_Acc / planer.Max_Jerk)
				{
						planer.acc_start[5] = planer.acc_end[4];
						planer.vel_start[5] = planer.vel_end[4];
						planer.vel_end[5] = planer.Max_Acc * planer.Max_Acc / planer.Max_Jerk;
					
						No_6_predictor(&planer);
						
						planer.acc_start[6] = planer.acc_end[5];
						planer.vel_start[6] = planer.vel_end[5];
						No_7_predictor(&planer);
				}
				else
				{
						planer.pos_deta[5] = 0;   									//��6�׶�Ԥ��Ϊ0
						planer.acc_start[6] = planer.acc_end[4];
						planer.vel_start[6] = planer.vel_end[4];
						No_7_predictor(&planer);
				}
				
				
				output_value = (pos + planer.pos_deta[2] + planer.pos_deta[4] + planer.pos_deta[5] + planer.pos_deta[6]);
				
				if(myABS(acc) >= planer.Max_Acc)			//���ٶȵ��Ｋ�ޣ������2�׶�
				{
						acc = myLIMIT(acc,-planer.Max_Acc,planer.Max_Acc);			//���Ƽ��ٶȵ������ٶ�
					
						step_now = 2;			
				}
				else if(output_value >= input_value)
				{
//						pos -= vel;
//						vel -= acc;
//						acc -= planer.Max_Jerk;				//�Ե�ǰ�ٶȡ����ٶ�Ԥ�ⳬ���趨�����˵���һ����״̬
						
						step_now = 3;									//�����3�׶�
				}
				
				
		}else 
		
		
		if(step_now == 2)			//��2�׶�����
		{
				vel += acc;
				pos += vel;
			
				planer.acc_start[2] = acc;
				planer.vel_start[2] = vel;
				No_3_predictor(&planer);
			
				if(myABS(planer.vel_end[2]) >= planer.Max_Vel)		//Ԥ���3�׶�ĩ�ٶ��ѳ������ֵ���ȼ��ٶν����������3�׶�
				{		
						vel = get_No_3_MAX_Vs(&planer);								//��������ٶȼ�������3�׶ε������ٶȡ���Ϊ��ֵ�����Ż���
						
						step_now = 3;			
				}
				else
				{
						planer.acc_start[4] = 0;
						planer.vel_start[4] = planer.vel_end[2];
						No_5_predictor(&planer);
					
						if(myABS(planer.vel_end[4]) > planer.Max_Acc * planer.Max_Acc / planer.Max_Jerk)
						{
								planer.acc_start[5] = planer.acc_end[4];
								planer.vel_start[5] = planer.vel_end[4];
								planer.vel_end[5] = planer.Max_Acc * planer.Max_Acc / planer.Max_Jerk;
							
								No_6_predictor(&planer);
								
								planer.acc_start[6] = planer.acc_end[5];
								planer.vel_start[6] = planer.vel_end[5];
								No_7_predictor(&planer);
						}
						else
						{
								planer.pos_deta[5] = 0;   									//��6�׶�Ԥ��Ϊ0
								planer.acc_start[6] = planer.acc_end[4];
								planer.vel_start[6] = planer.vel_end[4];
								No_7_predictor(&planer);
						}
						
						
						output_value = (pos + planer.pos_deta[2] + planer.pos_deta[4] + planer.pos_deta[5] + planer.pos_deta[6]);
						
						if(output_value >= input_value)
						{
//								pos -= vel;
//								vel -= acc;				//�Ե�ǰ�ٶȡ����ٶ�Ԥ�ⳬ���趨�����˵���һ����״̬
								
								step_now = 3;									//�����3�׶�
						}
				}
			
		}else
		
		
		if(step_now == 3)				//��3�׶�����
		{
				acc -= planer.Max_Jerk;
				vel += acc;
				pos += vel;
			
				if(acc <= 0)
				{
						if(myABS(planer.vel_end[2]) >= planer.Max_Vel)		//Ԥ���3�׶�ĩ�ٶ��ѳ������ֵ���ȼ��ٶν����������3�׶�
						{
								step_now = 4;
								acc = 0;
						}
						else
						{
								step_now = 5;
								acc = 0;
						}
				}
				
				
			}else
		
			if(step_now == 4)
			{
					acc = 0;
					vel = planer.Max_Vel;
					pos += vel;
				
					if(step_4_pre_computer == 0)
					{
						
						
							planer.acc_start[4] = 0;
							planer.vel_start[4] = vel;
							No_5_predictor(&planer);
						
							if(myABS(planer.vel_end[4]) > planer.Max_Acc * planer.Max_Acc / planer.Max_Jerk)
							{
									planer.acc_start[5] = planer.acc_end[4];
									planer.vel_start[5] = planer.vel_end[4];
									planer.vel_end[5] = planer.Max_Acc * planer.Max_Acc / planer.Max_Jerk;
								
									No_6_predictor(&planer);
									
									planer.acc_start[6] = planer.acc_end[5];
									planer.vel_start[6] = planer.vel_end[5];
									No_7_predictor(&planer);
							}
							else
							{
									planer.pos_deta[5] = 0;   									//��6�׶�Ԥ��Ϊ0
									planer.acc_start[6] = planer.acc_end[4];
									planer.vel_start[6] = planer.vel_end[4];
									No_7_predictor(&planer);
							}
							
							step_4_pre_computer = 1;
					}
					
					output_value = (pos + planer.pos_deta[4] + planer.pos_deta[5] + planer.pos_deta[6]);
					
					if(output_value >= input_value)
					{
						
							step_4_pre_computer = 0;			//���Ԥ�����־
							step_now = 5;									//�����5�׶�
					}
					
			}else
			
			
			if(step_now == 5)
			{
					acc -= planer.Max_Jerk;
					vel += acc;
					pos += vel;
				
					if(step_5_pre_computer == 0)
					{
							planer.acc_start[4] = 0;
							planer.vel_start[4] = vel;
							No_5_predictor(&planer);
						
							step_5_pre_computer = 1;
					}
					
					if(myABS(acc) >= myABS(planer.acc_end[4]))
					{
							acc = planer.acc_end[4];
						
							step_5_pre_computer = 0;
						
						
							if(myABS(planer.vel_end[4]) > planer.Max_Acc * planer.Max_Acc / planer.Max_Jerk)
									step_now = 6;									//�����6�׶�
							else
									step_now = 7;									//�����7�׶�
					}
				
			}else
			
			
			if(step_now == 6)
			{
					vel += acc;
					pos += vel;
				
				
					planer.acc_start[6] = acc;
					planer.vel_start[6] = vel;
					No_7_predictor(&planer);

					output_value = (pos + planer.pos_deta[6]);
				
					if(output_value >= input_value)
					{
							step_now = 7;									//�����7�׶�
							
					}

				
			}else
			
			if(step_now == 7)
			{
					acc += planer.Max_Jerk;
					vel += acc;
					pos += vel;
				
					if(acc > 0)
					{
							step_now = 0;
					}
			}else
			
			if(step_now == 0)
			{
					
					
			}
		
		
		
		
		planer_output = pos;
		MotorControl.pos.set_pos = planer_output / 1000;
		
}




