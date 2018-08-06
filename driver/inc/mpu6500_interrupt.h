#ifndef __MPU6050_INTERRUPT_H__
#define __MPU6050_INTERRUPT_H__
#include "stm32f4xx.h"

void MPU6500_IntConfiguration(void);
extern uint8_t isMPU6500_is_DRY;

#endif
