#ifndef __DRIVER_PanController_H
#define __DRIVER_PanController_H

#include "stm32f4xx.h"

typedef struct {
    float TargetVX;
    float TargetVY;
    float TargetWR;
    int   WheelSpeed[4];
} ChassisParam_Type;

void Chassis_Limit_Wheel_Speed(int WheelSpeedOrigin[4], int WheelSpeedRes[4], int MaxWheelSpeed);

void Chassis_Set_Wheel_Speed(int XSpeed, int YSpeed, int WSpeed);

void Chassis_Init_Yaw_Angle(void);

void Chassis_Update_Mecanum_Data(int buffer[4]);

#endif
