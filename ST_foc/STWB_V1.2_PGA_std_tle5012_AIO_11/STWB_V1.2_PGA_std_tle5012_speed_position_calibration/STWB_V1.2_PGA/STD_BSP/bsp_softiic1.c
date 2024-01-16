#include "bsp_softiic1.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_rcc.h"

u8 I2C1_FastMode;

//ģ��IIC��ʼ��
void I2C1_Soft_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_I2C1, ENABLE );
    GPIO_InitStructure.GPIO_Pin =  I2C1_Pin_SCL | I2C1_Pin_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(I2C1_PORT, &GPIO_InitStructure);
}


void I2C1_Soft_Delay(void)
{
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();


    if(!I2C1_FastMode)
    {
        u8 i = 8;
        while(i--);
    }
}

int I2C1_Soft_Start(void)
{
    SDA1_H;
    SCL1_H;
    I2C1_Soft_Delay();
    if(!SDA1_read)return 0;	//SDA��Ϊ�͵�ƽ������æ,�˳�
    SDA1_L;
    I2C1_Soft_Delay();
    if(SDA1_read) return 0;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
    SDA1_L;
    I2C1_Soft_Delay();
    return 1;
}

void I2C1_Soft_Stop(void)
{
    SCL1_L;
    I2C1_Soft_Delay();
    SDA1_L;
    I2C1_Soft_Delay();
    SCL1_H;
    I2C1_Soft_Delay();
    SDA1_H;
    I2C1_Soft_Delay();
}

void I2C1_Soft_Ack(void)
{
    SCL1_L;
    I2C1_Soft_Delay();
    SDA1_L;
    I2C1_Soft_Delay();
    SCL1_H;
    I2C1_Soft_Delay();
    SCL1_L;
    I2C1_Soft_Delay();
}

void I2C1_Soft_NoAck(void)
{
    SCL1_L;
    I2C1_Soft_Delay();
    SDA1_H;
    I2C1_Soft_Delay();
    SCL1_H;
    I2C1_Soft_Delay();
    SCL1_L;
    I2C1_Soft_Delay();
}

int I2C1_Soft_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
    SCL1_L;
    I2C1_Soft_Delay();
    SDA1_H;
    I2C1_Soft_Delay();
    SCL1_H;
    I2C1_Soft_Delay();
    if(SDA1_read)
    {
        SCL1_L;
        I2C1_Soft_Delay();
        return 0;
    }
    SCL1_L;
    I2C1_Soft_Delay();
    return 1;
}

void I2C1_Soft_SendByte(u8 SendByte)
{
    u8 i=8;
    while(i--)
    {
        SCL1_L;
        I2C1_Soft_Delay();
        if(SendByte&0x80)
            SDA1_H;
        else
            SDA1_L;
        SendByte<<=1;
        I2C1_Soft_Delay();
        SCL1_H;
        I2C1_Soft_Delay();
    }
    SCL1_L;
}

u8 I2C1_Soft_ReadByte(void)  //���ݴӸ�λ����λ//
{
    u8 i=8;
    u8 ReceiveByte=0;

    SDA1_H;
    while(i--)
    {
        ReceiveByte<<=1;
        SCL1_L;
        I2C1_Soft_Delay();
        SCL1_H;
        I2C1_Soft_Delay();
        if(SDA1_read)
        {
            ReceiveByte|=0x01;
        }
    }
    SCL1_L;
    return ReceiveByte;
}

//���ֽ�д��
int I2C1_Soft_Single_Write(u8 SlaveAddress,u8 REG_Address,u8 REG_data)
{
    if(!I2C1_Soft_Start())return 0;
    I2C1_Soft_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//���ø���ʼ��ַ+������ַ
    if(!I2C1_Soft_WaitAck()) {
        I2C1_Soft_Stop();
        return 0;
    }
    I2C1_Soft_SendByte(REG_Address );   //���õ���ʼ��ַ
    I2C1_Soft_WaitAck();
    I2C1_Soft_SendByte(REG_data);
    I2C1_Soft_WaitAck();
    I2C1_Soft_Stop();
    return 1;
}

//���ֽڶ�ȡ
int I2C1_Soft_Single_Read(u8 SlaveAddress,u8 REG_Address)
{
    unsigned char REG_data;
    if(!I2C1_Soft_Start())return 0;
    I2C1_Soft_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ
    if(!I2C1_Soft_WaitAck())
    {
        I2C1_Soft_Stop();
        return 0;
    }
    I2C1_Soft_SendByte((u8) REG_Address);   //���õ���ʼ��ַ
    I2C1_Soft_WaitAck();
    I2C1_Soft_Start();
    I2C1_Soft_SendByte(SlaveAddress+1);
    I2C1_Soft_WaitAck();

    REG_data= I2C1_Soft_ReadByte();
    I2C1_Soft_NoAck();
    I2C1_Soft_Stop();
    return REG_data;
}

//���ֽڶ�ȡ
int I2C1_Soft_Mult_Read(u8 SlaveAddress,u8 REG_Address,u8 * ptChar,u8 size)
{
    uint8_t i;

    if(size < 1)
        return 0;
    if(!I2C1_Soft_Start())
        return 0;
    I2C1_Soft_SendByte(SlaveAddress);
    if(!I2C1_Soft_WaitAck())
    {
        I2C1_Soft_Stop();
        return 0;
    }
    I2C1_Soft_SendByte(REG_Address);
    I2C1_Soft_WaitAck();

    I2C1_Soft_Start();
    I2C1_Soft_SendByte(SlaveAddress+1);
    I2C1_Soft_WaitAck();

    for(i=1; i<size; i++)
    {
        *ptChar++ = I2C1_Soft_ReadByte();
        I2C1_Soft_Ack();
    }
    *ptChar++ = I2C1_Soft_ReadByte();
    I2C1_Soft_NoAck();
    I2C1_Soft_Stop();
    return 1;
}
