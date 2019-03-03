#ifndef __DRIVER_MATH_H
#define __DRIVER_MATH_H

#include "stm32f4xx.h"

typedef struct {
    int value;
		int lastValue;
		
		int maxLimit;
		int minLimit;
		int counter;
		
	
} Ramp_Lowpass_Type;

void Ramp_Lowpass_Init(Ramp_Lowpass_Type *filter);
void Power_Ramp(Ramp_Lowpass_Type *filter, int start, int stop);
void Power_Lowpass(Ramp_Lowpass_Type *filter, int powerset);

#endif