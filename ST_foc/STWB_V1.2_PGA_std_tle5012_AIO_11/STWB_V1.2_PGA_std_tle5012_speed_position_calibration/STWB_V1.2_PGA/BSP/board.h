#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/


#include "stm32f3xx_hal.h"

#include "std_config.h"


//#include "mc_spi.h"
//#include "as5048.h"


void DelayUs(uint32_t us);

void DelayMs(uint32_t ms);

void TDT_Board_ALL_Init(void);

uint32_t GetSysTime_us(void);

void CurrentsCalibration(void);


extern u8 Init_OK;

extern volatile uint32_t sysTickUptime;

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
