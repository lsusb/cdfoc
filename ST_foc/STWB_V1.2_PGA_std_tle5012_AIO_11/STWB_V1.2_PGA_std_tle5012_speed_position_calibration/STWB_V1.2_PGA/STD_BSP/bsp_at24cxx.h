#ifndef __BSP_AT24CXX_H__
#define __BSP_AT24CXX_H__

#include "std_config.h"

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	    8191
#define AT24C128	16383
#define AT24C256	32767  


#define EE_TYPE AT24C02


//格式转换联合体
typedef union
{
    unsigned char U[4];
    float F;
    unsigned short I;
} FormatTrans;


u8 AT24CXX_ReadOneByte(u16 ReadAddr);							//指定地址读取一个字节
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite);		//指定地址写入一个字节
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len);//指定地址开始写入指定长度的数据
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len);					//指定地址开始读取指定长度数据
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite);	//从指定地址开始写入指定长度的数据
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead);   	//从指定地址开始读出指定长度的数据

u8 AT24CXX_Check(void);  //检查器件
void At24c02_Init(void);

void EEPROM_WriteParameter(void);
void EEPROM_ReadParameter(void);


void u16TOu8(uint16_t offest,u8* savebuffer,uint16_t data);
void u8TOu16(uint16_t offest,u8* savebuffer,uint16_t* data);

void s16TOu8(uint16_t offest,u8* savebuffer,int16_t data);
void u8TOs16(uint16_t offest,u8* savebuffer,int16_t* data);

void floatTou8(uint16_t offest,u8* savebuffer,float data);
void u8Tofloat(uint16_t offest,u8* savebuffer,float* data);




#endif
