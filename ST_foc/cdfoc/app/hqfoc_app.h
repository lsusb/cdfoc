#ifndef __HQFOC_H__
#define __HQFOC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "std_config.h"

void drv_write_reg(uint8_t reg, uint16_t val);
uint16_t drv_read_reg(uint8_t reg);
void drv_init(void);

void tle_5012_init(void);
uint16_t encoder_read(void);
void encoder_isr_prepare(void);

uint16_t encoder_reg_r(uint8_t addr);
void encoder_reg_w(uint8_t addr, uint16_t val);

uint16_t HQ_FOC_CurrControllerM1(void);
void SPI1_DMA_RX_CPT_Callback(void);
void encoder_isr(void);
uint16_t tle_5012_DMA_Read(void);

#ifdef __cplusplus
}
#endif

#endif // __HQFOC_H__
