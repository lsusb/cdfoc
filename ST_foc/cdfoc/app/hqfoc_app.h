#ifndef __HQFOC_H__
#define __HQFOC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "std_config.h"

typedef enum {
	POS_FRAME,  //位置数据帧
	MAG_FRAME  //磁场强度数据帧
} ams5311_frame_type_e;


void drv_write_reg(uint8_t reg, uint16_t val);
uint16_t drv_read_reg(uint8_t reg);
void drv_init(void);



uint16_t HQ_FOC_CurrControllerM1(void);



uint32_t ams5311_read(ams5311_frame_type_e frame_type);
void change_spi1_cpol(uint8_t cpol);



#ifdef __cplusplus
}
#endif

#endif // __HQFOC_H__
