#ifndef __DRIVER_CHASSIS_H
#define __DRIVER_CHASSIS_H

#include "stm32f4xx.h"

typedef struct {
    float TargetVX;
    float TargetVY;
    float TargetWR;
    int   WheelSpeed[4];
} ChassisParam_Type;

void Chassis_Set_Speed(float XSpeed, float YSpeed, float WSpeed);

void Chassis_Get_Rotor_Speed(int rotorSpeed[4]);

void Chassis_Limit_Rotor_Speed(int wheelSpeed[4], int rotorSpeed[4]);

#endif
