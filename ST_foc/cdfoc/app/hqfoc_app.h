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

#ifdef __cplusplus
}
#endif

#endif // __HQFOC_H__
