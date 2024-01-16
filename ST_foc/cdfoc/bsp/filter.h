#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

#include "std_config.h"
#define M_PI_F 3.1415926f


typedef struct{
	float raw_value;
	float xbuf[18];
	float ybuf[18];
	float filtered_value;
}Filter_t;


typedef struct{
	long long raw_value;
	long long sum_buffer;	
	long long filtered_value;
	u8			 cnt;
}AverageFilter_t;


float Chebyshev50HzLPF(Filter_t *F);
long long AverageFilter(AverageFilter_t *filter,u8 sum_cnt);
float SlideAverageFilter(int raw_data);

void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq);
float LPF2pApply_1(float sample);

void LPF2pSetCutoffFreq_2(float sample_freq, float cutoff_freq);
float LPF2pApply_2(float sample);

extern int16_t MecAngle2EleAngle[65536];



#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
