#ifndef __HQFOC_H__
#define __HQFOC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "std_config.h"

#define POLE_PITCH 12.5         //磁极距离，单位mm
#define ENC_COUNT_PER_MM 2048   //每mm多少CNT    
#define ENC_COUNT_PER_POLE_PAIR (int32_t)(25 * ENC_COUNT_PER_MM)
#define EANGLE_MANGLE_RATE 65536 / ENC_COUNT_PER_POLE_PAIR

    typedef enum
    {
        POS_FRAME, // 位置数据帧
        MAG_FRAME  // 磁场强度数据帧
    } ams5311_frame_type_e;

    typedef struct as5311_pack
    {
        _Bool even_par;
        _Bool mag_dec;
        _Bool mag_inc;
        _Bool lin;
        _Bool cof;
        _Bool ocf;

        int16_t posdata;
        int16_t magdata;
        int16_t ElAngle;

        int32_t overloop_pos;


        float pos_mm;
    } as5311_pack_t, *as5311_pack_p;

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
