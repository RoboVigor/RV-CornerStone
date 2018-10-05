/**
 * @file Driver_Chassis.h
 * @brief 底盘运动驱动
 * @version 0.5
 * - Can_Send() 新增了电调标识符id参数,所有CAN发送只需要这一个函数就能够完成
 */

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
