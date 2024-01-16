#ifndef __AS5048A_H
#define __AS5048A_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "std_config.h"

#define AS5048_ANGLE					0xFFFF

#define AS5048_SLIDEWINDOW_NUM 16

typedef struct _as5048
{
    uint16_t       reg;
		uint16_t			 reg_cal;
		long long			 pos;
		long long			 pos_slidewindow[AS5048_SLIDEWINDOW_NUM];
		long  				 pos_slidewindow_cnt;
		long long 		 pos_slidewindow_output;
	
		long 					 cnt;
		int16_t       	 ElectricAngle;
		
    u8     ef;
    long pos_sum;
		float angle;
    float       speed;
    float       speed_ef;
    long int    speed_jscop;
	
	
} as5048_t;



#define  RCC_SPI3_CS           RCC_AHBPeriph_GPIOA
#define  SPI3_CS_PORT          GPIOA
#define  SPI3_CS_Pin     			 GPIO_Pin_15


#define  SPI3_CS_LOW      SPI3_CS_PORT->BRR = SPI3_CS_Pin
#define  SPI3_CS_HIGH     SPI3_CS_PORT->BSRR = SPI3_CS_Pin
#define  SPI3_CS_TOGGLE 	SPI3_CS_PORT->ODR ^= SPI3_CS_Pin

    
void AS5048_Init(void);
void SPI_DMA_WRITE_READ_BUF(void);




#ifdef __cplusplus
}
#endif

#endif /* __AS5048A_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
