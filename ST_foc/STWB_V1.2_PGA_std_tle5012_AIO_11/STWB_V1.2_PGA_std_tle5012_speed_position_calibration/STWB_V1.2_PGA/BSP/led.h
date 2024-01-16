#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "std_config.h"


/***************LED GPIO¶¨Òå******************/
#define  LED_PORT          GPIOA
#define  GREEN_LED_Pin     GPIO_PIN_4


#define  LED_GREEN_ON       HAL_GPIO_WritePin(LED_PORT,GREEN_LED_Pin,GPIO_PIN_RESET)	
#define  LED_GREEN_OFF      HAL_GPIO_WritePin(LED_PORT,GREEN_LED_Pin,GPIO_PIN_SET)	
#define  LED_GREEN_TOGGLE   LED_PORT->ODR ^= GREEN_LED_Pin


    
    




#ifdef __cplusplus
}
#endif

#endif /* __LED_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
