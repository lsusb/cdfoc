#include "bsp_key.h"

#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
#include "stm32f30x_rcc.h"


void KEY_Init(void)
{
	
		GPIO_InitTypeDef  GPIO_InitStructure;

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; 
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOE2,3,4
	
} 


u8 Get_keyValue(void)
{
		_Bool key[3];
		u8 val;
		key[0] = !GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
		key[1] = !GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14);
		key[2] = !GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15);
		val = (key[0]<<2) | (key[1]<<1) | key[2];
		return val;
}


