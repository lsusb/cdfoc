#include "bsp_at24cxx.h"
#include "bsp_softiic1.h"
#include "board.h"

#include "control.h" 

void At24c02_Init(void)
{
		I2C1_Soft_Init();
}

//��AT24CXXָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{				  
		u8 temp=0;		  	    																 
		I2C1_Soft_Start();  
		if(EE_TYPE>AT24C16)
		{
				I2C1_Soft_SendByte(0XA0);	   //����д����
				I2C1_Soft_WaitAck();
				I2C1_Soft_SendByte(ReadAddr>>8);//���͸ߵ�ַ
				I2C1_Soft_WaitAck();		 
		}
		else 
				I2C1_Soft_SendByte(0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д���� 	 

		I2C1_Soft_WaitAck(); 
		I2C1_Soft_SendByte(ReadAddr%256);   //���͵͵�ַ
		I2C1_Soft_WaitAck();	    
		I2C1_Soft_Start();  	 	   
		I2C1_Soft_SendByte(0XA1);           //�������ģʽ			   
		I2C1_Soft_WaitAck();	 
		temp=I2C1_Soft_ReadByte();	
		I2C1_Soft_NoAck();
		I2C1_Soft_Stop();//����һ��ֹͣ����	    
		return temp;
		
}


//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{				   	  	    																 
		I2C1_Soft_Start();  
		if(EE_TYPE>AT24C16)
		{
				I2C1_Soft_SendByte(0XA0);	    //����д����
				I2C1_Soft_WaitAck();
				I2C1_Soft_SendByte(WriteAddr>>8);//���͸ߵ�ַ
		}else
		{
				I2C1_Soft_SendByte(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д���� 
		}	 
		I2C1_Soft_WaitAck();	   
		I2C1_Soft_SendByte(WriteAddr%256);   //���͵͵�ַ
		I2C1_Soft_WaitAck(); 	 										  		   
		I2C1_Soft_SendByte(DataToWrite);     //�����ֽ�							   
		I2C1_Soft_WaitAck();  		    	   
		I2C1_Soft_Stop();//����һ��ֹͣ���� 
		DelayMs(10);	 
}



//��AT24CXX�����ָ����ַ��ʼд�볤��ΪLen������
//�ú�������д��16bit����32bit������.
//WriteAddr  :��ʼд��ĵ�ַ  
//DataToWrite:���������׵�ַ
//Len        :Ҫд�����ݵĳ���2,4
void AT24CXX_WriteLenByte(u16 WriteAddr,u32 DataToWrite,u8 Len)
{  	
		u8 t;
		for(t=0;t<Len;t++)
		{
				AT24CXX_WriteOneByte(WriteAddr+t,(DataToWrite>>(8*t))&0xff);
		}												    
}

//��AT24CXX�����ָ����ַ��ʼ��������ΪLen������
//�ú������ڶ���16bit����32bit������.
//ReadAddr   :��ʼ�����ĵ�ַ 
//����ֵ     :����
//Len        :Ҫ�������ݵĳ���2,4
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
//���AT24CXX�Ƿ�����
//��������24XX�����һ����ַ(255)���洢��־��.
//���������24Cϵ��,�����ַҪ�޸�
//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
		u8 temp;
		temp=AT24CXX_ReadOneByte(255);//����ÿ�ο�����дAT24CXX			   
		if(temp==0X55)return 0;		   
		else//�ų���һ�γ�ʼ�������
		{
				AT24CXX_WriteOneByte(255,0X55);
				temp=AT24CXX_ReadOneByte(255);	  
				if(temp==0X55)return 0;
		}
		return 1;											  
}

//��AT24CXX�����ָ����ַ��ʼ����ָ������������
//ReadAddr :��ʼ�����ĵ�ַ ��24c02Ϊ0~255
//pBuffer  :���������׵�ַ
//NumToRead:Ҫ�������ݵĸ���
void AT24CXX_Read(u16 ReadAddr,u8 *pBuffer,u16 NumToRead)
{
		while(NumToRead)
		{
				*pBuffer++=AT24CXX_ReadOneByte(ReadAddr++);	
				NumToRead--;
		}
}  
//��AT24CXX�����ָ����ַ��ʼд��ָ������������
//WriteAddr :��ʼд��ĵ�ַ ��24c02Ϊ0~255
//pBuffer   :���������׵�ַ
//NumToWrite:Ҫд�����ݵĸ���
void AT24CXX_Write(u16 WriteAddr,u8 *pBuffer,u16 NumToWrite)
{
		while(NumToWrite--)
		{
				AT24CXX_WriteOneByte(WriteAddr,*pBuffer);
				WriteAddr++;
				pBuffer++;
		}
}



//�޷���16λ���//
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


//�з���16λ���//
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


//�����ȸ�����//
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

		u16TOu8(0,save_buffer,MotorControl.info.version_number);  				//����汾�ţ���ַ0-1��
		save_buffer[2] = MotorControl.info.mag_alignment;									//У׼��־λ����ַ2��
		s16TOu8(3,save_buffer,MotorControl.parameter.pos_compensation);		//��Ƕ���λ�궨ֵ����ַ3-4��
	
		save_buffer[5] = MotorControl.parameter.control_mode;							//����ģʽ����ַ5
	
		s16TOu8(6,save_buffer,MotorControl.parameter.pos_parameter.kp);		//λ�û�KP����ַ6-7��
		s16TOu8(8,save_buffer,MotorControl.parameter.pos_parameter.ki);		//λ�û�KI����ַ8-9��
	
		s16TOu8(10,save_buffer,MotorControl.parameter.speed_parameter.kp);		//�ٶȻ�KP����ַ10-11��
		s16TOu8(12,save_buffer,MotorControl.parameter.speed_parameter.ki);		//�ٶȻ�KI����ַ12-13��
	
		u16TOu8(14,save_buffer,MotorControl.parameter.max_angle);							//���Ƕȣ���ַ14-15��
		u16TOu8(16,save_buffer,MotorControl.parameter.max_speed);							//����ٶȣ���ַ16-17��
	
		AT24CXX_Write(0,save_buffer,18);
		
}

void EEPROM_ReadParameter(void)
{
	
		u8 read_buffer[50];
	
		AT24CXX_Read(0,read_buffer,18);
	
		u8TOu16(0,read_buffer,&MotorControl.info.version_number_eeprom);					//����汾�ţ���ַ0-1��
	
		if(MotorControl.info.version_number_eeprom == MotorControl.info.version_number)			//���Flash�ڵİ汾�ţ���̼�һ�²Ŷ�ȡ����
		{
				MotorControl.info.mag_alignment = read_buffer[2];										//У׼��־λ����ַ2��
				u8TOs16(3,read_buffer,&MotorControl.parameter.pos_compensation);		//��Ƕ���λ�궨ֵ����ַ3-4��
				
				MotorControl.parameter.control_mode = read_buffer[5];								//����ģʽ����ַ5
			
				u8TOs16(6,read_buffer,&MotorControl.parameter.pos_parameter.kp);		//λ�û�KP����ַ6-7��
				u8TOs16(8,read_buffer,&MotorControl.parameter.pos_parameter.ki);		//λ�û�KI����ַ8-9��
			
				u8TOs16(10,read_buffer,&MotorControl.parameter.speed_parameter.kp);	//�ٶȻ�KP����ַ10-11��
				u8TOs16(12,read_buffer,&MotorControl.parameter.speed_parameter.ki);	//�ٶȻ�KI����ַ12-13��
			
				u8TOu16(14,read_buffer,&MotorControl.parameter.max_angle);					//���Ƕȣ���ַ14-15��
				u8TOu16(16,read_buffer,&MotorControl.parameter.max_speed);					//����ٶȣ���ַ16-17��
			
		}
		
}

