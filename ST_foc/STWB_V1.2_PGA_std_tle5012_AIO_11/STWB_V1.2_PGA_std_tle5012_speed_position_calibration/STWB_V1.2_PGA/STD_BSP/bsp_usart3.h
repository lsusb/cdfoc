/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_USART3_H
#define __BSP_USART3_H

/* Includes ------------------------------------------------------------------*/
#include "std_config.h"

/* Exported types ------------------------------------------------------------*/
typedef struct _usart_RxMsg
{
	int16_t set_pos;
	int16_t	pc_cmd;
} usart_RxMsg_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define USART3_TX_DATASIZE 22

extern usart_RxMsg_t usart_RxMsg;
extern char usart3_tx_buffer[USART3_TX_DATASIZE];

/* Exported functions ------------------------------------------------------- */
void Usart3_Init(void);
void Usart3_SendChar(unsigned char b);
void Uart3_SendStr(char*SendBuf);
void Usart3_send_DMA(void);


#endif /* __BSP_USART3_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
