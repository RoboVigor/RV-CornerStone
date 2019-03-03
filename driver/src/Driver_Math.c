#define __DRIVER_MATH_GLOBALS

#include "Driver_Math.h"
#include "handle.h"

void Ramp_Lowpass_Init(Ramp_Lowpass_Type *filter) {
  filter->value = 0;
  filter->lastValue = 0;
  filter->maxLimit = 0;
  filter->minLimit = 0;
  filter->counter = 0;
}

void Power_Ramp(Ramp_Lowpass_Type *filter, int start, int stop) {
  float step;
  step = (float)(stop - start) / 100.0;

  filter->value = (int)(start + step * filter->counter);
  if (filter->value >= stop) {
    filter->value = stop;
  }

  if (filter->counter < 10000) {
    filter->counter++;
  }
}

void Power_Lowpass(Ramp_Lowpass_Type *filter, int powerset) {

  filter->value = powerset * 100 * 0.02 + filter->lastValue * (1 - 0.02);
  filter->lastValue = filter->value;
}
