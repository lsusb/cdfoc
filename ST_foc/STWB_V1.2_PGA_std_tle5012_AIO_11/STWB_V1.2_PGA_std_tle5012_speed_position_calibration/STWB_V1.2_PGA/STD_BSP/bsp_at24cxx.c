#include "bsp_at24cxx.h"
#include "bsp_softiic1.h"
#include "board.h"

#include "control.h" 

void At24c02_Init(void)
{
		I2C1_Soft_Init();
}

//在AT24CXX指定地址读出一个数据
//ReadAddr:开始读数的地址  
//返回值  :读到的数据
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
		u8 temp=0;		  	    																 
		I2C1_Soft_Start();  
		if(EE_TYPE>AT24C16)
		{
				I2C1_Soft_SendByte(0XA0);	   //发送写命令
				I2C1_Soft_WaitAck();
				I2C1_Soft_SendByte(ReadAddr>>8);//发送高地址
				I2C1_Soft_WaitAck();		 
		}
		else 
				I2C1_Soft_SendByte(0XA0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据 	 

		I2C1_Soft_WaitAck(); 
		I2C1_Soft_SendByte(ReadAddr%256);   //发送低地址
		I2C1_Soft_WaitAck();	    
		I2C1_Soft_Start();  	 	   
		I2C1_Soft_SendByte(0XA1);           //进入接收模式			   
		I2C1_Soft_WaitAck();	 
		temp=I2C1_Soft_ReadByte();	
		I2C1_Soft_NoAck();
		I2C1_Soft_Stop();//产生一个停止条件	    
		return temp;
		
}


//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址    
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
		I2C1_Soft_Start();  
		if(EE_TYPE>AT24C16)
		{
				I2C1_Soft_SendByte(0XA0);	    //发送写命令
				I2C1_Soft_WaitAck();
				I2C1_Soft_SendByte(WriteAddr>>8);//发送高地址
		}else
		{
				I2C1_Soft_SendByte(0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据 
		}	 
		I2C1_Soft_WaitAck();	   
		I2C1_Soft_SendByte(WriteAddr%256);   //发送低地址
		I2C1_Soft_WaitAck(); 	 										  		   
		I2C1_Soft_SendByte(DataToWrite);     //发送字节							   
		I2C1_Soft_WaitAck();  		    	   
		I2C1_Soft_Stop();//产生一个停止条件 
		DelayMs(10);	 
}



//在AT24CXX里面的指定地址开始写入长度为Len的数据
//该函数用于写入16bit或者32bit的数据.
//WriteAddr  :开始写入的地址  
//DataToWrite:数据数组首地址
//Len        :要写入数据的长度2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
		u8 t;
		for(t=0;t<Len;t++)
		{
				AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
		}												    
}

//在AT24CXX里面的指定地址开始读出长度为Len的数据
//该函数用于读出16bit或者32bit的数据.
//ReadAddr   :开始读出的地址 
//返回值     :数据
//Len        :要读出数据的长度2,4
u32 AT24CXX_ReadLenByte(u16 ReadAddr,u8 Len)
{  	
		u8 t;
		u32 temp=0;
		for(t=0;t<Len;t++)
		{
				temp<<=8;
				temp+=AT24CXX_ReadOneByte(ReadAddr+Len-t-1); 	 				   
		}
		return temp;												    
}
//检查AT24CXX是否正常
//这里用了24XX的最后一个地址(255)来存储标志字.
//如果用其他24C系列,这个地址要修改
//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
		u8 temp;
		temp=AT24CXX_ReadOneByte(255);//避免每次开机都写AT24CXX			   
		if(temp==0X55)return 0;		   
		else//排除第一次初始化的情况
		{
				AT24CXX_WriteOneByte(255,0X55);
				temp=AT24CXX_ReadOneByte(255);	  
				if(temp==0X55)return 0;
		}
		return 1;											  
}

//在AT24CXX里面的指定地址开始读出指定个数的数据
//ReadAddr :开始读出的地址 对24c02为0~255
//pBuffer  :数据数组首地址
//NumToRead:要读出数据的个数
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
		while(NumToRead)
		{
				*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
				NumToRead--;
		}
}  
//在AT24CXX里面的指定地址开始写入指定个数的数据
//WriteAddr :开始写入的地址 对24c02为0~255
//pBuffer   :数据数组首地址
//NumToWrite:要写入数据的个数
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
		while(NumToWrite--)
		{
				AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
				WriteAddr++;
				pBuffer++;
		}
}



//无符号16位拆合//
void u16TOu8(uint16_t offest,u8* savebuffer,uint16_t data)
{
		savebuffer += offest;
		*savebuffer = (u8)(data >> 8);
		savebuffer +=1;
		*savebuffer = (u8)(data);
}
void u8TOu16(uint16_t offest,u8* savebuffer,uint16_t* data)
{
		u8 temp[2];
		savebuffer += offest+1;
		temp[1] = *savebuffer;
		savebuffer -= 1;
		temp[0] = *savebuffer;
		
		*data = (uint16_t)(temp[1] | temp[0]<<8);
}


//有符号16位拆合//
void s16TOu8(uint16_t offest,u8* savebuffer,int16_t data)
{
		savebuffer += offest;
		*savebuffer = (u8)(data >> 8);
		savebuffer +=1;
		*savebuffer = (u8)(data);
}
void u8TOs16(uint16_t offest,u8* savebuffer,int16_t* data)
{
		u8 temp[2];
		savebuffer += offest+1;
		temp[1] = *savebuffer;
		savebuffer -= 1;
		temp[0] = *savebuffer;
		
		*data = (int16_t)(temp[1] | temp[0]<<8);
}


//单精度浮点拆合//
void floatTou8(uint16_t offest,u8* savebuffer,float data)
{
		FormatTrans temp;
	
		temp.F = data;
	
		savebuffer += offest;
		*savebuffer = temp.U[0];
		savebuffer +=1;
		*savebuffer = temp.U[1];
		savebuffer +=1;
		*savebuffer = temp.U[2];
		savebuffer +=1;
		*savebuffer = temp.U[3];
}
void u8Tofloat(uint16_t offest,u8* savebuffer,float* data)
{
		FormatTrans temp;
		savebuffer += offest;
	
		temp.U[0] = *savebuffer;
		savebuffer +=1;
		temp.U[1] = *savebuffer;
		savebuffer +=1;
		temp.U[2] = *savebuffer;
		savebuffer +=1;
		temp.U[3] = *savebuffer;
	
		*data = temp.F;
}





void EEPROM_WriteParameter(void)
{
		u8 save_buffer[50];

		u16TOu8(0,save_buffer,MotorControl.info.version_number);  				//软件版本号，地址0-1；
		save_buffer[2] = MotorControl.info.mag_alignment;									//校准标志位，地址2；
		s16TOu8(3,save_buffer,MotorControl.parameter.pos_compensation);		//电角度相位标定值，地址3-4；
	
		save_buffer[5] = MotorControl.parameter.control_mode;							//控制模式，地址5
	
		s16TOu8(6,save_buffer,MotorControl.parameter.pos_parameter.kp);		//位置环KP，地址6-7；
		s16TOu8(8,save_buffer,MotorControl.parameter.pos_parameter.ki);		//位置环KI，地址8-9；
	
		s16TOu8(10,save_buffer,MotorControl.parameter.speed_parameter.kp);		//速度环KP，地址10-11；
		s16TOu8(12,save_buffer,MotorControl.parameter.speed_parameter.ki);		//速度环KI，地址12-13；
	
		u16TOu8(14,save_buffer,MotorControl.parameter.max_angle);							//最大角度，地址14-15；
		u16TOu8(16,save_buffer,MotorControl.parameter.max_speed);							//最大速度，地址16-17；
	
		AT24CXX_Write(0,save_buffer,18);
		
}

void EEPROM_ReadParameter(void)
{
	
		u8 read_buffer[50];
	
		AT24CXX_Read(0,read_buffer,18);
	
		u8TOu16(0,read_buffer,&MotorControl.info.version_number_eeprom);					//软件版本号，地址0-1；
	
		if(MotorControl.info.version_number_eeprom == MotorControl.info.version_number)			//检测Flash内的版本号，与固件一致才读取参数
		{
				MotorControl.info.mag_alignment = read_buffer[2];										//校准标志位，地址2；
				u8TOs16(3,read_buffer,&MotorControl.parameter.pos_compensation);		//电角度相位标定值，地址3-4；
				
				MotorControl.parameter.control_mode = read_buffer[5];								//控制模式，地址5
			
				u8TOs16(6,read_buffer,&MotorControl.parameter.pos_parameter.kp);		//位置环KP，地址6-7；
				u8TOs16(8,read_buffer,&MotorControl.parameter.pos_parameter.ki);		//位置环KI，地址8-9；
			
				u8TOs16(10,read_buffer,&MotorControl.parameter.speed_parameter.kp);	//速度环KP，地址10-11；
				u8TOs16(12,read_buffer,&MotorControl.parameter.speed_parameter.ki);	//速度环KI，地址12-13；
			
				u8TOu16(14,read_buffer,&MotorControl.parameter.max_angle);					//最大角度，地址14-15；
				u8TOu16(16,read_buffer,&MotorControl.parameter.max_speed);					//最大速度，地址16-17；
			
		}
		
}

