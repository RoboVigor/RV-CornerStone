#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "stm32f4xx.h"

void BSP_TIM_Init(void);

void TIM4_PWM_Init(void);

#ifdef  __BSP_TIM_Global
#define __BSP_TIM_EXT
#else
#define __BSP_TIM_EXT extern
#endif

__BSP_TIM_EXT volatile uint32_t ulHighFrequencyTimerTicks;

#endif
