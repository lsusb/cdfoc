#include "AS5048A.h"
#include "bsp_spi.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_spi.h"
#include "stm32f30x_dma.h"

void AS5048_Init(void)
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


//SPI_DMA ��дһ��buf
#define BufSize		1
void SPI_DMA_WRITE_READ_BUF(void) 
{   
	
		TX_Buff[0] = AS5048_ANGLE;
		
    // �رշ��� DMA	
    DMA_Cmd(DMA2_Channel2, DISABLE);	
		DMA_Cmd(DMA2_Channel1, DISABLE);	
    // ���÷��͵�������
    DMA_SetCurrDataCounter(DMA2_Channel2, BufSize);
		DMA_SetCurrDataCounter(DMA2_Channel1, 2);
	// �������
    SPI3->DR;			  	
    // ����DMA��־λ
    DMA_ClearFlag(DMA2_IT_TC2);	
		DMA_ClearFlag(DMA2_IT_TC1);
    // Ƭѡ����,��������
    SPI3_CS_LOW;	
		// �������� DMA
    DMA_Cmd(DMA2_Channel2, ENABLE);
		DMA_Cmd(DMA2_Channel1, ENABLE);
	
}






