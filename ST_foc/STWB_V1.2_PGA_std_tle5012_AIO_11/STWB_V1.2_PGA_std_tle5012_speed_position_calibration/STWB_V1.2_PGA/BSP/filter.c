#include "filter.h"
#include "mc_type.h"

#include "arm_math.h"



const float NUM[18] = {
  0.0004793484404,   -0.002149262, 0.005930194166,    -0.01084904,  0.01534511615,
   -0.01655694097,   0.0145611912,-0.009326857515, 0.003402658505, 0.003402658505,
  -0.009326857515,   0.0145611912, -0.01655694097,  0.01534511615,    -0.01084904,
   0.005930194166,   -0.002149262,0.0004793484404
};

const float DEN[18] = {
                1,   -8.624773979,    36.39727783,    -99.0246048,    193.5945892,
     -287.6271973,    335.4121704,   -312.9385986,    236.1031342,   -144.6713867,
      71.91122437,   -28.80280495,    9.174246788,   -2.273334026,   0.4229340255,
   -0.05563091859,  0.00461706752,-0.0001817250886
};

float Chebyshev50HzLPF(Filter_t *F)
{
	int i;
	for(i=17; i>0; i--)
	{
		F->ybuf[i] = F->ybuf[i-1]; 
		F->xbuf[i
		] = F->xbuf[i-1];
	}
	F->xbuf[0] = F->raw_value;
	F->ybuf[0] = NUM[0] * F->xbuf[0];
	for(i=1;i<18;i++)
	{
		F->ybuf[0] = F->ybuf[0] + NUM[i] * F->xbuf[i] - DEN[i] * F->ybuf[i];
	}
	F->filtered_value = F->ybuf[0];
	return F->filtered_value;
}


long long AverageFilter(AverageFilter_t *filter,u8 sum_cnt)
{
		filter->sum_buffer += filter->raw_value;
		filter->cnt ++;
		if(filter->cnt >= sum_cnt)
		{
				filter->filtered_value = filter->sum_buffer/filter->cnt;
				filter->cnt = 0;
				filter->sum_buffer = 0;
		}
		return filter->filtered_value;
}

int16_t MecAngle2EleAngle[65536];

#define SLIDEWINDOW_LENGTH 20
float SlideAverageFilter(int raw_data)
{
		static int	SlideWindow[SLIDEWINDOW_LENGTH];
		static long data_sum = 0;
	
		float result;
		
		for(u8 i=0;i<SLIDEWINDOW_LENGTH-1;i++)
		{
				SlideWindow[i] = SlideWindow[i+1];
				data_sum += SlideWindow[i];
		}
		SlideWindow[SLIDEWINDOW_LENGTH-1] = raw_data;
		data_sum += raw_data;
		result = (float)data_sum/(float)SLIDEWINDOW_LENGTH;
		data_sum = 0;
		return result;
}


#define SLIDEWINDOW_IQ_LENGTH 30
int16_t SlideAverageFilter_iq(int16_t raw_data)
{
		static int16_t	SlideWindow[SLIDEWINDOW_IQ_LENGTH];
		static long data_sum = 0;
	
		int16_t result;
		
		for(u8 i=0;i<SLIDEWINDOW_IQ_LENGTH-1;i++)
		{
				SlideWindow[i] = SlideWindow[i+1];
				data_sum += SlideWindow[i];
		}
		SlideWindow[SLIDEWINDOW_IQ_LENGTH-1] = raw_data;
		data_sum += raw_data;
		result = (float)data_sum/(float)SLIDEWINDOW_IQ_LENGTH;
		data_sum = 0;
		return result;
}





static float           _cutoff_freq1;
static float           _a11;
static float           _a21;
static float           _b01;
static float           _b11;
static float           _b21;
static float           _delay_element_11;        // buffered sample -1
static float           _delay_element_21;        // buffered sample -2
void LPF2pSetCutoffFreq_1(float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq1 = cutoff_freq;
    if (_cutoff_freq1 > 0.0f)
    {
        _b01 = ohm*ohm/c;
        _b11 = 2.0f*_b01;
        _b21 = _b01;
        _a11 = 2.0f*(ohm*ohm-1.0f)/c;
        _a21 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LPF2pApply_1(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq1 <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_11 * _a11 - _delay_element_21 * _a21;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b01 + _delay_element_11 * _b11 + _delay_element_21 * _b21;

        _delay_element_21 = _delay_element_11;
        _delay_element_11 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}



static float           _cutoff_freq2;
static float           _a12;
static float           _a22;
static float           _b02;
static float           _b12;
static float           _b22;
static float           _delay_element_12;        // buffered sample -1
static float           _delay_element_22;        // buffered sample -2
void LPF2pSetCutoffFreq_2(float sample_freq, float cutoff_freq)
{
    float fr =0;
    float ohm =0;
    float c =0;

    fr= sample_freq/cutoff_freq;
    ohm=tanf(M_PI_F/fr);
    c=1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;

    _cutoff_freq2 = cutoff_freq;
    if (_cutoff_freq2 > 0.0f)
    {
        _b02 = ohm*ohm/c;
        _b12 = 2.0f*_b02;
        _b22 = _b02;
        _a12 = 2.0f*(ohm*ohm-1.0f)/c;
        _a22 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
    }
}

float LPF2pApply_2(float sample)
{

    float delay_element_0 = 0, output=0;
    if (_cutoff_freq2 <= 0.0f) {
        // no filtering
        return sample;
    }
    else
    {
        delay_element_0 = sample - _delay_element_12 * _a12 - _delay_element_22 * _a22;
        // do the filtering
        if (isnan(delay_element_0) || isinf(delay_element_0)) {
            // don't allow bad values to propogate via the filter
            delay_element_0 = sample;
        }
        output = delay_element_0 * _b02 + _delay_element_12 * _b12 + _delay_element_22 * _b22;

        _delay_element_22 = _delay_element_12;
        _delay_element_12 = delay_element_0;

        // return the value.  Should be no need to check limits
        return output;
    }
}
