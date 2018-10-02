#ifndef __DRIVER_CAN_H
#define __DRIVER_CAN_H

#include "stm32f4xx.h"

void Can_Transmit(CAN_TypeDef *CANx, int16_t stdId, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);

#endif
