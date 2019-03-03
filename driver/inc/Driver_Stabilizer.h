/**
 * @file Stabilizer.h
 * @brief 云台初始化和设置角度函数
 * @version 0.1
 */

#ifndef __DRIVER_STABILIZER_H
#define __DRIVER_STABILIZER_H

#include "stm32f4xx.h"

#define PITCH_MAXANGLE 30.0f
#define PITCH_MINANGLE -25.0f

typedef struct {
    float anglePosition;
    float targetAngle;
    float angularSpeed;
} Stabilizer_Type;

void Stabilizer_Init_Course_Angle(Stabilizer_Type *CourseAngle);

void Stabilizer_Set_TargetAngle(float pitch_control, float yaw_control);

#endif
