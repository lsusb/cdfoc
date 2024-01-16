#ifndef _PARAMETER_H
#define	_PARAMETER_H

#include "std_config.h"


#define pidCurrent_Id 		0
#define pidCurrent_Iq 		1
#define pidSpeed			 		2
#define pidPos				 		3
#define gm3510Speed			 	4
#define gm3510Pos				 	5



/**
* @struct  _pid
* @brief pid½á¹¹Ìå
*/
typedef struct _pid
{
	float setValue;
	float feedbackValue;
	float error;
	float lastError;
	float deltaError;
	float integralError;
	float integralErrorMax;
	float kp;
	float ki;
	float kd;
	float pOut;
	float iOut;
	float dOut;
	float out;
}pid;




void Get_PIDparameters(pid* pidStruct, u8 pidIndex);


#endif
