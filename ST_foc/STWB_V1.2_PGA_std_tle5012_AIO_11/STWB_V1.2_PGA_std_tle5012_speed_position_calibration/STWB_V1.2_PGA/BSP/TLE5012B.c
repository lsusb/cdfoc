#include "TLE5012B.h"
#include "spi.h"
extern volatile uint16_t TLE5012_reg;


uint16_t ang_reg_v = 0x8021, data_v;
void Read_TLE5012(void)
{
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
 
    HAL_SPI_Transmit_DMA(&hspi3, (uint8_t *)(&ang_reg_v), 1);

    HAL_SPI_Receive_DMA(&hspi3, (uint8_t *)(&data_v), 1);
 
		
//		HAL_SPI_TransmitReceive(&hspi1,(uint8_t *)(&ang_reg_v),(uint8_t *)(&data_v),1,0XFF);
	
    TLE5012_reg = (uint16_t)((data_v & 0x7fff));
	
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	
 
}
