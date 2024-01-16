#include "bsp_spi.h"
#include "AS5048A.h"
#include "filter.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_spi.h"
#include "stm32f30x_dma.h"

void SPI3_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
	
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);//ʹ��GPIOAʱ��
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
	
    //GPIOB5,3��ʼ������
	
    /**SPI3 GPIO Configuration
    PB3     ------> SPI3_SCK
    PB5     ------> SPI3_MOSI
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;		
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			
    GPIO_Init(GPIOB, &GPIO_InitStructure);								//��ʼ��

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_6); 		//PB3     ------> SPI3_SCK
//    GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_6); 		
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_6); 		//PB5     ------> SPI3_MOSI
		
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//��λSPI1
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//ֹͣ��λSPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;  
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//����SPI�����ݴ�С:SPI���ͽ���16λ֡�ṹ
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�ߵ�ƽ
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//����ͬ��ʱ�ӵĵڶ��������أ��������½������ݱ�����
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ4
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
    SPI_Init(SPI3, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

		SPI_RxFIFOThresholdConfig(SPI3, SPI_RxFIFOThreshold_HF);
		
    SPI_Cmd(SPI3, ENABLE); //ʹ��SPI����
		
//    DMA_DeInit(DMA2_Channel1);
//    DMA_DeInit(DMA2_Channel2);
//		
//		SPI3_TX_DMA_Config();
//		SPI3_RX_DMA_Config();
//		
//		
//		//SPI2 TX DMA����ʹ��
//		SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);	
//		//SPI2 RX DMA����ʹ��
//		SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);	
		
		
}
#define SENDBUFF_SIZE (1)	    // һ�η��͵�����	
uint16_t TX_Buff[2];		// ���ͻ���
void SPI3_TX_DMA_Config(void)
{
    // �жϽṹ��
    NVIC_InitTypeDef NVIC_InitStructure;		
    // DMA�ṹ��
    DMA_InitTypeDef DMA_InitStructure;  		
    /* ʹ��DMAʱ�� */  		
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
    /* ��λ��ʼ��DMA������ */  
    DMA_DeInit(DMA2_Channel2);								

    /* ���� DMA Stream */
    /* �����ַ */  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR;	
    /* �ڴ��ַ(Ҫ����ı�����ָ��) ,DMA�洢��0��ַ*/  	
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TX_Buff;	
    /* ���򣺴洢�������� */			
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    /* ���ݴ����� ,������Ϊ0�� ʵ�ʷ���ʱ����������*/	    
    DMA_InitStructure.DMA_BufferSize = (uint32_t)SENDBUFF_SIZE;		
    /* ���������ģʽ */		
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    /* �洢������ģʽ */  	
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* �������ݳ���:16λ */	 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    /* �ڴ����ݳ���:16λ */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    /* DMAģʽ������ģʽ */  		
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* ���ȼ����� */	 		
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    /* ��ʼ��DMA Stream */		
    DMA_Init(DMA2_Channel2, &DMA_InitStructure);
    /* ������������ж�  */		
    DMA_ITConfig(DMA2_Channel2,DMA_IT_TC,ENABLE);

    // �жϳ�ʼ�� 
    /* DMA�����ж�Դ */  
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel2_IRQn;	
    /* �������ȼ� */  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    /* ��Ӧ���ȼ� */  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
    /* ʹ���ⲿ�ж�ͨ�� */ 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						 
    /* ����NVIC */		
    NVIC_Init(&NVIC_InitStructure);
}


//DMA�����жϷ�����
void DMA2_Channel2_IRQHandler(void)
{
    // DMA �������
    if(DMA_GetITStatus(DMA2_IT_TC2))
    {
			
			
        // ���DMA������ɱ�־
        DMA_ClearITPendingBit(DMA2_IT_TC2);	
    }
}

#define RECEIVE_SIZE  		2  	// ���մ�С
uint16_t RX_Buff[2];		// ���յ�����
void SPI3_RX_DMA_Config(void)
{
    // �жϽṹ��
    NVIC_InitTypeDef NVIC_InitStructure;	
    // DMA�ṹ��  
    DMA_InitTypeDef DMA_InitStructure;		
    /* ʹ��DMAʱ��*/  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);					/* ��λ��ʼ��DMA������ */ 
    DMA_DeInit(DMA2_Channel1);												/* ȷ��DMA��������λ��� */

    /* ����DMAԴ���������ݼĴ�����ַ*/  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR		;
    /* �ڴ��ַ(Ҫ����ı�����ָ��)*/  
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RX_Buff;			
    /* ���򣺴洢��������ģʽ */			
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    /* ���ݴ����� ,��Ҫ�����ܽ��ܵ�������[����Ϊ0],ʵ�ʷ���ʱ����������*/	  
    DMA_InitStructure.DMA_BufferSize = (uint32_t)RECEIVE_SIZE;
    /* ���������ģʽ */	  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 	  
    /* �洢������ģʽ */    
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* �������ݳ���:16λ */	  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    /* �ڴ����ݳ���16λ */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;		
    /* DMAģʽ������ģʽ */  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* ���ȼ����� */	   
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    /* ��ʼ��DMA Stream */		
    DMA_Init(DMA2_Channel1, &DMA_InitStructure);							   
    /* ������������ж�  */
    DMA_ITConfig(DMA2_Channel1,DMA_IT_TC,ENABLE);  					

    // �жϳ�ʼ�� 
    /* ���� DMA����Ϊ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel1_IRQn;  	
    /* �������ȼ� */  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		
    /* ��Ӧ���ȼ� */  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
    /* ʹ���ⲿ�ж�ͨ�� */  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* ����NVIC */	
    NVIC_Init(&NVIC_InitStructure);
		
		DMA_Cmd(DMA2_Channel1, ENABLE);

}

uint16_t re_5048;

volatile as5048_t senser_as5048;
AverageFilter_t as5048_pos_filter;
void DMA2_Channel1_IRQHandler(void)
{					
		static uint16_t as5048_reg_last = 0;
    // DMA�������
    if(DMA_GetITStatus(DMA2_IT_TC1))
    {		
        // ���ݽ������ ����Ƭѡ
        SPI3_CS_HIGH;	
        // ���DMA������ɱ�־λ		
        DMA_ClearITPendingBit(DMA2_IT_TC1);	
				
				re_5048 = RX_Buff[0]&0x3fff;
			
				senser_as5048.reg = RX_Buff[0]&0x3fff;
			
				if(as5048_reg_last - senser_as5048.reg > 16384/2)
						senser_as5048.cnt ++;
				if(as5048_reg_last - senser_as5048.reg < -16384/2)
						senser_as5048.cnt --;
				senser_as5048.pos = senser_as5048.reg + senser_as5048.cnt * 16384;
				
				as5048_pos_filter.raw_value = senser_as5048.pos;
				
				senser_as5048.pos_slidewindow_output = AverageFilter(&as5048_pos_filter,16);
				
///*******��������*************************************/				
//				for(u8 i=AS5048_SLIDEWINDOW_NUM-1;i>0;i--)
//					senser_as5048.pos_slidewindow[i] = senser_as5048.pos_slidewindow[i-1];

//				senser_as5048.pos_slidewindow[0] = senser_as5048.pos;
//				
//				for(u8 i=0;i<AS5048_SLIDEWINDOW_NUM;i++)
//				{
//						pos_slidewindow_temp += senser_as5048.pos_slidewindow[i];
//				}
//				
//				senser_as5048.pos_slidewindow_output = pos_slidewindow_temp/AS5048_SLIDEWINDOW_NUM;
//				
///****************************************************/				
						

				
//				if(senser_as5048.reg >= roter_pos_conpensation)
//				{
//						senser_as5048.reg_cal = senser_as5048.reg - roter_pos_conpensation;
//				}else
//				{
//						senser_as5048.reg_cal = 16384 - roter_pos_conpensation + senser_as5048.reg;
//				}
//				
//				senser_as5048.ElectricAngle = ((senser_as5048.reg_cal*11 % 16384) - 8192) * 4;  //��е�ǶȻ���Ƕ�
				
//				senser_as5048.ElectricAngle = (senser_as5048.reg_cal*11 % 16384)/16384.0f *360.0f;  //��е�ǶȻ���Ƕ�
//				
//				mc_svpm.Park.Theta = (uint16_t)(senser_as5048.ElectricAngle * 10.0f);
//			
//				mc_svpm.Park.Theta = LIMIT(mc_svpm.Park.Theta,0,3599);
			
				as5048_reg_last = senser_as5048.reg;
			
    }
}


uint16_t SPI3_ReadWrite16Bit(uint16_t Txdata)
{
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE) == RESET) {}
    SPI_I2S_SendData16(SPI3,Txdata);
    while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE) == RESET) {}
    return SPI_I2S_ReceiveData16(SPI3);
}


