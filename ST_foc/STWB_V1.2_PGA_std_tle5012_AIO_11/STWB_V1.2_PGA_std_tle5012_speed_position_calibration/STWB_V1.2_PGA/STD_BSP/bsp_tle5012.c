#include "bsp_tle5012.h"
#include "bsp_spi.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_spi.h"

void Tle5012_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_SPI3_CS, ENABLE );
    GPIO_InitStructure.GPIO_Pin =  SPI3_CS_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_Init(SPI3_CS_PORT,&GPIO_InitStructure);

    SPI3_CS_HIGH;
}




volatile uint16_t TLE5012_reg;
volatile uint16_t TLE5012_reg_14;

volatile int16_t TLE5012_speed;
reg_tle5012b_t Tle5012_Register;

void Read_TLE5012_STD(void)
{
		uint16_t ang_reg_v = 0x8021, data_v;
	
		SPI3_CS_LOW;
	
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData16(SPI3,ang_reg_v);
		while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET){}; //wait finish sending			
			
		while (SPI_GetReceptionFIFOStatus(SPI3) != SPI_ReceptionFIFOStatus_Empty)
			SPI_I2S_ReceiveData16(SPI3);			

		SPI_BiDirectionalLineConfig(SPI3,SPI_Direction_Rx);
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET) {};
		SPI3_CS_HIGH;
			
		SPI_BiDirectionalLineConfig(SPI3,SPI_Direction_Tx);
			
    data_v = SPI_I2S_ReceiveData16(SPI3);
			
    TLE5012_reg = (uint16_t)((data_v & 0x7fff));
		TLE5012_reg_14 = 	((data_v & 0x7fff))>>1;
}


void Read_TLE5012_STD_STATE_REG(void)
{
		uint16_t ang_reg_v = 0x8001, data_v;
	
		SPI3_CS_LOW;
	
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData16(SPI3,ang_reg_v);
		while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET){}; //wait finish sending			
			
		while (SPI_GetReceptionFIFOStatus(SPI3) != SPI_ReceptionFIFOStatus_Empty)
			SPI_I2S_ReceiveData16(SPI3);			

		SPI_BiDirectionalLineConfig(SPI3,SPI_Direction_Rx);
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET) {};
		SPI3_CS_HIGH;
			
		SPI_BiDirectionalLineConfig(SPI3,SPI_Direction_Tx);
			
    data_v = SPI_I2S_ReceiveData16(SPI3);
		Tle5012_Register.STATE_REG = data_v;
    Tle5012_Register.NO_GMR_A = (_Bool)(data_v&0x1000);
    Tle5012_Register.NO_GMR_XY = (_Bool)(data_v&0x800);
}


void Read_TLE5012_STD_SPEED(void)
{
	
		uint16_t ang_reg_v = 0x8031, data_v;
		SPI3_CS_LOW;
	
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData16(SPI3,ang_reg_v);
		while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET){}; //wait finish sending			
			
		while (SPI_GetReceptionFIFOStatus(SPI3) != SPI_ReceptionFIFOStatus_Empty)
			SPI_I2S_ReceiveData16(SPI3);			

		SPI_BiDirectionalLineConfig(SPI3,SPI_Direction_Rx);
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET) {};
		SPI3_CS_HIGH;
			
		SPI_BiDirectionalLineConfig(SPI3,SPI_Direction_Tx);
			
    data_v = SPI_I2S_ReceiveData16(SPI3);
			
		if(data_v & 0x4000)
		{
				TLE5012_speed = data_v|0xc000;
		}
		else
				TLE5012_speed = data_v & 0x7fff;
			
}


void Read_TLE5012_STD_Angle_Speed(void)
{
		uint16_t ang_reg_v = 0x8022, data_v;
	
		SPI3_CS_LOW;
	
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData16(SPI3,ang_reg_v);
		while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_BSY) == SET){}; //wait finish sending			
			
		while (SPI_GetReceptionFIFOStatus(SPI3) != SPI_ReceptionFIFOStatus_Empty)
			SPI_I2S_ReceiveData16(SPI3);			

		SPI_BiDirectionalLineConfig(SPI3,SPI_Direction_Rx);
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET) {};
			
    data_v = SPI_I2S_ReceiveData16(SPI3);
    TLE5012_reg = (uint16_t)((data_v & 0x7fff));
		TLE5012_reg_14 = 	((data_v & 0x7fff))>>1;
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET) {};
			
    data_v = SPI_I2S_ReceiveData16(SPI3);
		if(data_v & 0x4000)
		{
				TLE5012_speed = data_v|0xc000;
		}
		else
				TLE5012_speed = data_v & 0x7fff;
		
		SPI3_CS_HIGH;
			
		SPI_BiDirectionalLineConfig(SPI3,SPI_Direction_Tx);
			
			
}



