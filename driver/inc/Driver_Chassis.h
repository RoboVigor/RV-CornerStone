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
} ChassisData_Type;

void Chassis_Update(ChassisData_Type *ChassisData, float XSpeed, float YSpeed, float WSpeed);

void Chassis_Get_Rotor_Speed(ChassisData_Type *ChassisData, int rotorSpeed[4]);

void Chassis_Limit_Rotor_Speed(ChassisData_Type *ChassisData, int wheelSpeed[4], int rotorSpeed[4]);

#endif
