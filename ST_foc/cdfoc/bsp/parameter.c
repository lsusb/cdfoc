#include "parameter.h"
#include "control.h" 

//float kp_id = 2,ki_id = 15,kd_id,li_id = 2000;
//float kp_iq = 5,ki_iq = 15,kd_iq,li_iq = 2000;
//float kp_v = 70,ki_v = 0,kd_v,li_v = 0;
//float kp_pos = 0,ki_pos = 0,kd_pos,li_pos = 0;

//float kp_v3510 = 0,ki_v3510 = 0,li_v3510 = 0;
//float kp_p3510 = 100,ki_p3510 = 0,li_p3510 = 0;

//float kp_id = 2,ki_id = 2,kd_id,li_id = 15000;
//float kp_iq = 5,ki_iq = 5,kd_iq,li_iq = 6000;
//float kp_v = 40,ki_v = 4000,kd_v,li_v = 6;
//float kp_pos = 0.2,ki_pos = 0,kd_pos,li_pos = 0;

float kp_id = 2,ki_id = 2,kd_id,li_id = 15000;
float kp_iq = 5,ki_iq = 5,kd_iq,li_iq = 6000;
//float kp_v = 40,ki_v = 4000,kd_v,li_v = 6;
//float kp_pos = 0.2,ki_pos = 0,kd_pos,li_pos = 0;

float kp_v3510 = 0,ki_v3510 = 0,li_v3510 = 0;
float kp_p3510 = 100,ki_p3510 = 0,li_p3510 = 0;

void Get_PIDparameters(pid* pidStruct, u8 pidIndex)
{
    switch(pidIndex)
    {

				case pidCurrent_Id:
				{
						pidStruct->kp = kp_id;//20
						pidStruct->ki = ki_id;//4;
						pidStruct->kd = kd_id;
						pidStruct->integralErrorMax = li_id;
						break;
				}

				case pidCurrent_Iq:
				{
						pidStruct->kp = kp_iq;//20
						pidStruct->ki = ki_iq;//4;
						pidStruct->kd = kd_iq;
						pidStruct->integralErrorMax = li_iq;
						break;
				}			

				case pidSpeed:
				{
						pidStruct->kp = MotorControl.parameter.speed_parameter.kp;		//20
						pidStruct->ki = MotorControl.parameter.speed_parameter.ki;		//4;
						pidStruct->kd = MotorControl.parameter.speed_parameter.kd;
						pidStruct->integralErrorMax = MotorControl.parameter.speed_parameter.li;
						break;
				}			

				case pidPos:
				{
						pidStruct->kp = MotorControl.parameter.pos_parameter.kp / 100.0f;//20
						pidStruct->ki = MotorControl.parameter.pos_parameter.ki;//4;
						pidStruct->kd = MotorControl.parameter.pos_parameter.kd;
						pidStruct->integralErrorMax = MotorControl.parameter.pos_parameter.li;
						break;
				}				
				
				case gm3510Speed:
				{
						pidStruct->kp = kp_v3510;//20
						pidStruct->ki = ki_v3510;//4;
						pidStruct->kd = 0;
						pidStruct->integralErrorMax = li_v3510;
						break;
				}		
				
				case gm3510Pos:
				{
						pidStruct->kp = kp_p3510;//20
						pidStruct->ki = ki_p3510;//4;
						pidStruct->kd = 0;
						pidStruct->integralErrorMax = li_p3510;
						break;
				}				
				
    default:
        break;
    }
}



