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
	
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);//使能GPIOA时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
	
	
    //GPIOB5,3初始化设置
	
    /**SPI3 GPIO Configuration
    PB3     ------> SPI3_SCK
    PB5     ------> SPI3_MOSI
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;		
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			
    GPIO_Init(GPIOB, &GPIO_InitStructure);								//初始化

    GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_6); 		//PB3     ------> SPI3_SCK
//    GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_6); 		
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_6); 		//PB5     ------> SPI3_MOSI
		
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//复位SPI1
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI1

    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;  
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//设置SPI的数据大小:SPI发送接收16位帧结构
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;		//定义波特率预分频的值:波特率预分频值为4
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
    SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
    SPI_Init(SPI3, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

		SPI_RxFIFOThresholdConfig(SPI3, SPI_RxFIFOThreshold_HF);
		
    SPI_Cmd(SPI3, ENABLE); //使能SPI外设
		
//    DMA_DeInit(DMA2_Channel1);
//    DMA_DeInit(DMA2_Channel2);
//		
//		SPI3_TX_DMA_Config();
//		SPI3_RX_DMA_Config();
//		
//		
//		//SPI2 TX DMA请求使能
//		SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);	
//		//SPI2 RX DMA请求使能
//		SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);	
		
		
}
#define SENDBUFF_SIZE (1)	    // 一次发送的数据	
uint16_t TX_Buff[2];		// 发送缓存
void SPI3_TX_DMA_Config(void)
{
    // 中断结构体
    NVIC_InitTypeDef NVIC_InitStructure;		
    // DMA结构体
    DMA_InitTypeDef DMA_InitStructure;  		
    /* 使能DMA时钟 */  		
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);	
    /* 复位初始化DMA数据流 */  
    DMA_DeInit(DMA2_Channel2);								

    /* 配置 DMA Stream */
    /* 外设地址 */  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR;	
    /* 内存地址(要传输的变量的指针) ,DMA存储器0地址*/  	
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)TX_Buff;	
    /* 方向：存储器到外设 */			
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    /* 数据传输量 ,可设置为0， 实际发送时会重新设置*/	    
    DMA_InitStructure.DMA_BufferSize = (uint32_t)SENDBUFF_SIZE;		
    /* 外设非增量模式 */		
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    /* 存储器增量模式 */  	
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* 外设数据长度:16位 */	 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    /* 内存数据长度:16位 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    /* DMA模式：正常模式 */  		
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* 优先级：高 */	 		
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;

		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    /* 初始化DMA Stream */		
    DMA_Init(DMA2_Channel2, &DMA_InitStructure);
    /* 开启传输完成中断  */		
    DMA_ITConfig(DMA2_Channel2,DMA_IT_TC,ENABLE);

    // 中断初始化 
    /* DMA发送中断源 */  
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel2_IRQn;	
    /* 抢断优先级 */  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    /* 响应优先级 */  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
    /* 使能外部中断通道 */ 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						 
    /* 配置NVIC */		
    NVIC_Init(&NVIC_InitStructure);
}


//DMA发送中断服务函数
void DMA2_Channel2_IRQHandler(void)
{
    // DMA 发送完成
    if(DMA_GetITStatus(DMA2_IT_TC2))
    {
			
			
        // 清除DMA发送完成标志
        DMA_ClearITPendingBit(DMA2_IT_TC2);	
    }
}

#define RECEIVE_SIZE  		2  	// 接收大小
uint16_t RX_Buff[2];		// 接收到缓存
void SPI3_RX_DMA_Config(void)
{
    // 中断结构体
    NVIC_InitTypeDef NVIC_InitStructure;	
    // DMA结构体  
    DMA_InitTypeDef DMA_InitStructure;		
    /* 使能DMA时钟*/  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);					/* 复位初始化DMA数据流 */ 
    DMA_DeInit(DMA2_Channel1);												/* 确保DMA数据流复位完成 */

    /* 设置DMA源：串口数据寄存器地址*/  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&SPI3->DR		;
    /* 内存地址(要传输的变量的指针)*/  
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RX_Buff;			
    /* 方向：存储器到外设模式 */			
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    /* 数据传输量 ,需要最大可能接受的数据量[不能为0],实际发送时会重新设置*/	  
    DMA_InitStructure.DMA_BufferSize = (uint32_t)RECEIVE_SIZE;
    /* 外设非增量模式 */	  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 	  
    /* 存储器增量模式 */    
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* 外设数据长度:16位 */	  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    /* 内存数据长度16位 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;		
    /* DMA模式：正常模式 */  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* 优先级：高 */	   
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
		
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    /* 初始化DMA Stream */		
    DMA_Init(DMA2_Channel1, &DMA_InitStructure);							   
    /* 开启传输完成中断  */
    DMA_ITConfig(DMA2_Channel1,DMA_IT_TC,ENABLE);  					

    // 中断初始化 
    /* 配置 DMA接收为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel1_IRQn;  	
    /* 抢断优先级 */  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		
    /* 响应优先级 */  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;				
    /* 使能外部中断通道 */  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 配置NVIC */	
    NVIC_Init(&NVIC_InitStructure);
		
		DMA_Cmd(DMA2_Channel1, ENABLE);

}

uint16_t re_5048;

volatile as5048_t senser_as5048;
AverageFilter_t as5048_pos_filter;
void DMA2_Channel1_IRQHandler(void)
{					
		static uint16_t as5048_reg_last = 0;
    // DMA接收完成
    if(DMA_GetITStatus(DMA2_IT_TC1))
    {		
        // 数据接收完成 拉高片选
        SPI3_CS_HIGH;	
        // 清除DMA接收完成标志位		
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
				
///*******滑动窗口*************************************/				
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
//				senser_as5048.ElectricAngle = ((senser_as5048.reg_cal*11 % 16384) - 8192) * 4;  //机械角度换电角度
				
//				senser_as5048.ElectricAngle = (senser_as5048.reg_cal*11 % 16384)/16384.0f *360.0f;  //机械角度换电角度
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


