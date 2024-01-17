#ifndef _CONTROL_H
#define	_CONTROL_H

#include "std_config.h"     
#include "mc_type.h"


typedef enum
{
		eeprom_err = 1

} my_State_t;



typedef struct{
		float set_vel;
		float reg_speed;
		float feedbk_vel;
		float output;
	
		float speed_rpm;
}_mt_velcontrol;

typedef struct{
		float set_pos;
		float feedbk_pos;
		float output;
}_mt_poscontrol;

typedef struct{
		float VBus;
		u8    motor_init;
		u8		mag_alignment;
		bool	eeprom_s;
		
		u8		eeprom_write;
		u8		eeprom_read;
	
		u8 		can_id;
	
		u8		motor_driver_name;		//�������
		
	
		uint16_t	version_number_eeprom;		
		uint16_t	version_number;		
		
}_mt_info;

typedef struct{
		int16_t kp;
		int16_t ki;
		int16_t li;
		int16_t kd;
}_pid_parameter;


typedef struct{
		int16_t 				pos_compensation;
		u8							control_mode;											//0:����ģʽ        1���ٶ�ģʽ          2��λ��ģʽ
		
		int16_t					feed_back_position_zeropoint;			//��е��λƫ��
		
		uint16_t				max_angle;												//���ת��
		uint16_t				max_speed;												//���ת��
		uint16_t				feed_back_freq;										//����Ƶ��
	
		_pid_parameter 	speed_parameter;
		_pid_parameter	pos_parameter;
	
	
}_mt_parameter;


typedef struct{
		long long pos;
		float 		pos_filter;
		float 		speed;
}_mt_magnetic_sensor;

typedef struct{
		float v_error;
		float p_error;
}_mt_controller;

typedef struct{
	
		qd_t Iqd;
		qd_t Iqd_f;
	
		qd_t Vqd;
	
		ab_t Iab;
		ab_t Iab_f;
	
		alphabeta_t Ialphabeta;
		alphabeta_t	Valphabeta;

		int16_t hElAngle;
		int16_t hMecAngle;
	
}_mt_foc;

typedef struct{
		_mt_poscontrol 				pos;
		_mt_velcontrol 				vel;
		_mt_info       				info;
		_mt_foc				 				foc;
		_mt_magnetic_sensor		tle5012; 
		_mt_controller				controller;
		_mt_parameter					parameter;
		
}_mt_control;


typedef struct{
		int32_t			vel_start[7];
		int32_t			vel_end[7];
	
		int32_t			acc_start[7];
		int32_t			acc_end[7];
	
		int32_t			pos_deta[7];
	
		int16_t			Max_Jerk;
		int32_t			Max_Acc;
		int32_t			Max_Vel;
	
}_mt_setValue_planer;


void Foc_Control(void);


void Current_IdControl(int16_t* setValue, int16_t* fbValue, int16_t* result, float T);
void Current_IqControl(float* setValue, int16_t* fbValue, int16_t* result, float T);
void MotorControlApp(float T);
void Speed_Control(float* setValue, float* fbValue, float* result, float T);
void Position_Control(float* setValue, float* fbValue, float* result, float T);

void GM3510_Speed_Control(float* setValue, float* fbValue, float* result, float T);
void GM3510_Pos_Control(float* setValue, float* fbValue, float* result, float T);

void GM3510_Control(float T);
float HardLIMIT(float rawdata,float min,float max);

void UsartSendFeed_back(void);
void Auto_mag_alignment(void);

void set_value_planer(void);


extern _mt_control MotorControl;


#endif


