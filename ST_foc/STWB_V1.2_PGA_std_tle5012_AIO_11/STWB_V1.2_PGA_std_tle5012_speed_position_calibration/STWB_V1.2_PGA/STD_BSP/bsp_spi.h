/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_SPI_H
#define __BSP_SPI_H

/* Includes ------------------------------------------------------------------*/
#include "std_config.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
extern uint16_t TX_Buff[2];		// ·¢ËÍ»º´æ

/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */
void SPI3_Init(void);
void SPI3_TX_DMA_Config(void);
void SPI3_RX_DMA_Config(void);

uint16_t SPI3_ReadWrite16Bit(uint16_t Txdata);

#endif /* __BSP_SPI_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
