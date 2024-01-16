#include "communication.h"

#include "control.h" 
#include "bsp_usart3.h"
#include "bsp_at24cxx.h"


void usart_decode(u8* buffer)
{
		u8TOs16(6, buffer, &MotorControl.parameter.pos_parameter.kp);
		u8TOs16(8, buffer, &MotorControl.parameter.pos_parameter.ki);
	
		u8TOs16(10, buffer, &MotorControl.parameter.speed_parameter.kp);
		u8TOs16(12, buffer, &MotorControl.parameter.speed_parameter.ki);
	
		u8TOu16(14, buffer, &MotorControl.parameter.max_angle);
		u8TOu16(16, buffer, &MotorControl.parameter.max_speed);
	
		u8TOu16(18, buffer, &MotorControl.parameter.feed_back_freq);
	
		MotorControl.parameter.control_mode = buffer[20];
	
		MotorControl.info.eeprom_write = 1;
}



void usart_userCMD_decode(u8* buffer)
{
		if(buffer[2] == 0x02 && MotorControl.parameter.control_mode == 2)
			MotorControl.pos.set_pos = (int32_t)(buffer[6]<<24)|(buffer[5]<<16)|(buffer[4]<<8)|(buffer[3]);
		if(buffer[2] == 0x01 && MotorControl.parameter.control_mode == 1)
			MotorControl.vel.set_vel = (int16_t)(buffer[4]<<8)|(buffer[3]);
		if(buffer[2] == 0x00 && MotorControl.parameter.control_mode == 0)
			MotorControl.vel.output = (int16_t)(buffer[4]<<8)|(buffer[3]);
		
		MotorControl.pos.set_pos = LIMIT(MotorControl.pos.set_pos,-16384*100,16384*100);
		MotorControl.vel.set_vel = LIMIT(MotorControl.vel.set_vel,-82,82);
		MotorControl.vel.output  = LIMIT(MotorControl.vel.output,-32767,32767);
		
}


void can_rx_decode(u8* buffer,u8 can_id)
{
		if(can_id == 1 || can_id == 3 || can_id == 5 || can_id == 7)
		{
				if(MotorControl.parameter.control_mode == 2)
						MotorControl.pos.set_pos = (int32_t)((buffer[3]<<24)|(buffer[2]<<16)|(buffer[1]<<8)|(buffer[0]));
				if(MotorControl.parameter.control_mode == 1)
						MotorControl.vel.set_vel = (int16_t)(buffer[1]<<8)|(buffer[0]);
				if(MotorControl.parameter.control_mode == 0)
						MotorControl.vel.output = (int16_t)(buffer[1]<<8)|(buffer[0]);
		}
		if(can_id == 2 || can_id == 4 || can_id == 6)
		{
				if(MotorControl.parameter.control_mode == 2)
						MotorControl.pos.set_pos = (int32_t)((buffer[7]<<24)|(buffer[6]<<16)|(buffer[5]<<8)|(buffer[4]));
				if(MotorControl.parameter.control_mode == 1)
						MotorControl.vel.set_vel = (int16_t)(buffer[5]<<8)|(buffer[4]);
				if(MotorControl.parameter.control_mode == 0)
						MotorControl.vel.output = (int16_t)(buffer[5]<<8)|(buffer[4]);
		}		
}


extern volatile uint16_t TLE5012_reg;

void can_tx_encode(u8* buffer)
{
	
		buffer[0] = (u8)(int16_t)(TLE5012_reg);
		buffer[1] =(u8)((int16_t)(TLE5012_reg)>>8);
	
		buffer[2] = (u8)(int16_t)((int16_t)(MotorControl.tle5012.speed *100));
		buffer[3] =(u8)((int16_t)((int16_t)(MotorControl.tle5012.speed *100))>>8);
	
		buffer[4] = (u8)(int16_t)(MotorControl.foc.Iqd.q);
		buffer[5] =(u8)((int16_t)(MotorControl.foc.Iqd.q)>>8);
}
