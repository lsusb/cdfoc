/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_CAN_H
#define __BSP_CAN_H

/* Includes ------------------------------------------------------------------*/
#include "std_config.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
typedef struct _mainboard_cmd
{
	int16_t ch0;
	int16_t ch2;
	int16_t ch4;
	int16_t ch5;
	
	u8 demonstration_mode;
} mainboard_cmd;

typedef struct _can1_feedback
{
	int16_t GM3510_pos;
	int16_t GM3510_torque;
	long long GM3510_sumpos;
	
	float	GM3510_speed;
	float	GM3510_speed_lpf;
	
	mainboard_cmd cmd;
		
} can1_feedback;

extern can1_feedback can1Feedback;
extern u8 key_can_id;


/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void CAN1_Configuration(void);
void CanSendFeed_back(void);


void Get_demonstration_mode(int16_t sw1,int16_t sw2,u8* mode);

#endif /* __BSP_CAN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
