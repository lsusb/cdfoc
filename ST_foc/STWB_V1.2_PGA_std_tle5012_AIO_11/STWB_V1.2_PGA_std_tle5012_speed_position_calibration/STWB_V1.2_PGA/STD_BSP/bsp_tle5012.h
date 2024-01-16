#ifndef __BSP_TLE5012B_H
#define __BSP_TLE5012B_H


#ifdef __cplusplus
extern "C" {
#endif

#include "std_config.h"


#define  RCC_SPI3_CS           RCC_AHBPeriph_GPIOA
#define  SPI3_CS_PORT          GPIOA
#define  SPI3_CS_Pin     			 GPIO_Pin_15


#define  SPI3_CS_LOW      SPI3_CS_PORT->BRR = SPI3_CS_Pin
#define  SPI3_CS_HIGH     SPI3_CS_PORT->BSRR = SPI3_CS_Pin
#define  SPI3_CS_TOGGLE 	SPI3_CS_PORT->ODR ^= SPI3_CS_Pin


typedef struct _reg_tle5012b
{
		_Bool NO_GMR_A;
		_Bool NO_GMR_XY;
		uint16_t STATE_REG;
} reg_tle5012b_t;


void Read_TLE5012_STD(void);
void Read_TLE5012_STD_STATE_REG(void);
void Read_TLE5012_STD_SPEED(void);
void Read_TLE5012_STD_Angle_Speed(void);

void Tle5012_Init(void);


extern volatile uint16_t TLE5012_reg;


#ifdef __cplusplus
}
#endif

#endif


