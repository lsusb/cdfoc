#ifndef __STD_CONFIG_H
#define __STD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )


typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;





#ifdef __cplusplus
}
#endif

#endif /* __STD_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
