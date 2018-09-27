#ifndef __DRIVER_PanController_H
#define __DRIVER_PanController_H

#include "stm32f4xx.h"

typedef struct {
    float TargetVX;
    float TargetVY;
    float TargetWR;
    int   WheelSpeed[4];
} ChassisParam_Type;

void Chassis_Set_Speed(int XSpeed, int YSpeed, int WSpeed);

void Chassis_Update_Wheel_Speed(int result[4]);

void Chassis_Limit_Wheel_Speed(int WheelSpeedOrigin[4], int WheelSpeedRes[4], int MaxWheelSpeed);

#endif
