#ifndef __MY_MATH_H
#define __MY_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "std_config.h"

#define M_PI_F 3.1415926f

struct _LPF
{
    float    _cutoff_freq1;
    float           _a11;
    float           _a21;
    float           _b01;
    float           _b11;
    float           _b21;
    float           _delay_element_11;        // buffered sample -1
    float           _delay_element_21;        // buffered sample -2
};

typedef struct _LPF  LPF;

void lpf_k_init(void);
void LPF2pSetCutoffFreq(int index,float sample_freq, float cutoff_freq);
float LPF2pApply(int index,float sample);



void Anti_Park_Calc(void);
void Clarke_Trans(void);
void Park_Trans(void);




extern float SinTable[3600];


#ifdef __cplusplus
}
#endif

#endif /* __MY_MATH_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
