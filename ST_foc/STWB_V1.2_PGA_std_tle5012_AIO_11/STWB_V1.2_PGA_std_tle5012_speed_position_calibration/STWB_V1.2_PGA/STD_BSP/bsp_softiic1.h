#ifndef _BSP_SOFT_I2C1_H__
#define _BSP_SOFT_I2C1_H__

#include "std_config.h"

#define RCC_I2C1	     RCC_AHBPeriph_GPIOA
#define I2C1_PORT      GPIOA
#define I2C1_Pin_SCL   GPIO_Pin_11
#define I2C1_Pin_SDA   GPIO_Pin_12




#define SCL1_H         I2C1_PORT->BSRR = I2C1_Pin_SCL
#define SCL1_L         I2C1_PORT->BRR = I2C1_Pin_SCL
#define SDA1_H         I2C1_PORT->BSRR = I2C1_Pin_SDA
#define SDA1_L         I2C1_PORT->BRR = I2C1_Pin_SDA
#define SDA1_read      I2C1_PORT->IDR  & I2C1_Pin_SDA

void I2C1_Soft_Init(void);
void I2C1_Soft_Delay(void);
int I2C1_Soft_Start(void);
void I2C1_Soft_Stop(void);
void I2C1_Soft_Ack(void);
void I2C1_Soft_NoAck(void);
int I2C1_Soft_WaitAck(void); 	 //·µ»Ø:=1ÓÐACK,=0ÎÞACK
void I2C1_Soft_SendByte(u8 SendByte);
u8 I2C1_Soft_ReadByte(void);

int I2C1_Soft_Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
int I2C1_Soft_Single_Read(u8 SlaveAddress,u8 REG_Address);
int I2C1_Soft_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size);

extern u8 I2C1_FastMode;

#endif
