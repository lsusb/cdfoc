#include "bsp_can.h"
#include "communication.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_can.h"

u8 key_can_id = 0;

void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_9);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_9);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_5tq;
    can.CAN_BS2 = CAN_BS2_3tq;
    can.CAN_Prescaler = 4;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

    can_filter.CAN_FilterNumber=0;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}
can1_feedback can1Feedback;

void USB_LP_CAN_RX0_IRQHandler(void)
{
    CanRxMsg Can1RxMsg;

    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &Can1RxMsg);
    }
		
		if((key_can_id == 1|| key_can_id == 2) && Can1RxMsg.StdId == 0x301)
		{
				can_rx_decode(Can1RxMsg.Data,key_can_id);
		}
		if((key_can_id == 3|| key_can_id == 4) && Can1RxMsg.StdId == 0x302)
		{
				can_rx_decode(Can1RxMsg.Data,key_can_id);
		}    
		if((key_can_id == 5|| key_can_id == 6) && Can1RxMsg.StdId == 0x303)
		{
				can_rx_decode(Can1RxMsg.Data,key_can_id);
		}    
		if((key_can_id == 7) && Can1RxMsg.StdId == 0x304)
		{
				can_rx_decode(Can1RxMsg.Data,key_can_id);
		}
    
//		switch(Can1RxMsg.StdId)
//    {
//    case (0x400):
//				can1Feedback.cmd.ch0 = (int16_t)((Can1RxMsg.Data[0]<<8)|(Can1RxMsg.Data[1]));    //右摇杆横向   范围+-660
//				can1Feedback.cmd.ch2 = (int16_t)((Can1RxMsg.Data[2]<<8)|(Can1RxMsg.Data[3]));
//				can1Feedback.cmd.ch4 = Can1RxMsg.Data[4];
//				can1Feedback.cmd.ch5 = Can1RxMsg.Data[5];
//				Get_demonstration_mode(can1Feedback.cmd.ch4,can1Feedback.cmd.ch5,&can1Feedback.cmd.demonstration_mode);
//		
//				break;
//				
//    case 0x127:
//				break;
//	 case 0x401:
//				break;    
//	 case 0x159:
//				break;    
//	 case 0x151:
//				break;   
//	 default:
//        break;
//    }
}

void USB_HP_CAN_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}


void Get_demonstration_mode(int16_t sw1,int16_t sw2,u8* mode)
{
		*mode = (u8)((sw1-1)*3 + sw2);
}



void CanSendFeed_back(void)
{
		static float pos_set_f;
	  CanTxMsg SingleTxMsg;


		SingleTxMsg.IDE = 0;             //标准帧
		SingleTxMsg.RTR = 0;             //数据帧
		SingleTxMsg.DLC = 8;             //帧长度
		
		if(key_can_id == 1)
			SingleTxMsg.StdId =  0x211;      
		else if(key_can_id == 2)
			SingleTxMsg.StdId =  0x212;   
		else if(key_can_id == 3)
			SingleTxMsg.StdId =  0x213;   
		else if(key_can_id == 4)
			SingleTxMsg.StdId =  0x214;   
		else if(key_can_id == 5)
			SingleTxMsg.StdId =  0x215; 
		else if(key_can_id == 6)
			SingleTxMsg.StdId =  0x216;   
		else if(key_can_id == 7)
			SingleTxMsg.StdId =  0x217;   
		
		
		can_tx_encode(SingleTxMsg.Data);
	
			
		CAN_Transmit(CAN1,&SingleTxMsg);
	
		
}

