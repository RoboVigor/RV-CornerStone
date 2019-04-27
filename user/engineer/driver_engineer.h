/**
 * @file Driver_Engineer
 * @brief 工程舵机驱动
 */

#ifndef __DRIVER_ENGINEER_H
#define __DRIVER_ENGINEER_H

#include "stm32f4xx.h"

void Turn_Magazine(int State);
void Turn_Camera_Screen(int State);
void Turn_Camera_Rescue(int State);
void Turn_Motor_Transimission_On(int State);

#endif