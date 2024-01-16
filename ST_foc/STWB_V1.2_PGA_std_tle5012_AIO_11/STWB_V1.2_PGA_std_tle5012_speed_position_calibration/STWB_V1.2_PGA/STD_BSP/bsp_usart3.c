#include "bsp_usart3.h"
#include "crc.h"
#include "communication.h"



#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_dma.h"

unsigned char usart3_rx_buffer[22];
char usart3_tx_buffer[USART3_TX_DATASIZE];

void Usart3_Init(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_DMA1,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_7);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_7);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
	
    USART_DeInit(USART3);
    USART_InitStructure.USART_BaudRate = 230400;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3,&USART_InitStructure);
    USART_Cmd(USART3,ENABLE);

    USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);

    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);      
		
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DMA_DeInit(DMA1_Channel3);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->RDR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart3_rx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 22;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3,&DMA_InitStructure);

    DMA_Cmd(DMA1_Channel3,ENABLE);
		
		
    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->TDR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart3_tx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = USART3_TX_DATASIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2,&DMA_InitStructure);
		
    DMA_Cmd(DMA1_Channel2,ENABLE);
		
}

usart_RxMsg_t usart_RxMsg;
void USART3_IRQHandler(void)
{

    u16 i;

    if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        DMA_Cmd(DMA1_Channel3,DISABLE);

        i = USART3->ISR;
        i = USART3->RDR;
				
				if(usart3_rx_buffer[0] == 0x66 && usart3_rx_buffer[1] == 0x77 && Verify_CRC8_Check_Sum(usart3_rx_buffer,8))				//控制指令
				{	
						usart_userCMD_decode(usart3_rx_buffer);
				}
				else if(usart3_rx_buffer[0] == 0xAC && usart3_rx_buffer[1] == 0xBE && Verify_CRC8_Check_Sum(usart3_rx_buffer,6))
				{
						usart_RxMsg.pc_cmd = usart3_rx_buffer[2];
					
						if(usart_RxMsg.pc_cmd == 0x02)
						{
								usart_decode(usart3_rx_buffer);
								usart_RxMsg.pc_cmd = 0x01;
							
						}
				}
			
        DMA1_Channel3->CNDTR = 22;
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
        DMA_Cmd(DMA1_Channel3,ENABLE);
    }
}

/*单字节串口发送函数*/
void Usart3_SendChar(unsigned char b)
{
   while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
	 USART_SendData(USART3,b);
}



/*串口3打印数据*/
void Uart3_SendStr(char*SendBuf)
{
	while(*SendBuf)
	{
	  while((USART3->ISR&0X40)==0)
          {}//等待发送完成 
        USART3->TDR = (u8) *SendBuf; 
		SendBuf++;
	}
}

void Usart3_send_DMA(void)
{
		if(DMA_GetFlagStatus(DMA1_FLAG_TC2)!=RESET)//等待DMA2_Steam7传输完成
		{ 
				DMA_Cmd(DMA1_Channel2, DISABLE);                      //关闭DMA传输 
				
				DMA_ClearFlag(DMA1_FLAG_TC2);//清除DMA2_Steam7传输完成标志
				DMA_ClearITPendingBit(DMA1_FLAG_TC2);
				DMA_SetCurrDataCounter(DMA1_Channel2,USART3_TX_DATASIZE);          //数据传输量  
	 
				DMA_Cmd(DMA1_Channel2, ENABLE);                      //开启DMA传输 
		}

}


