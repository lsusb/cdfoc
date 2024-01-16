#ifndef __TLE5012B_H
#define __TLE5012B_H


#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f3xx_hal.h"


#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA





void Read_TLE5012(void);



#ifdef __cplusplus
}
#endif

#endif


