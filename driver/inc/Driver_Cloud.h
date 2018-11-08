/**
 * @file Driver_Cloud.h
 * @brief 云台初始化和设置角度函数
 * @version 0.1
 */

#ifndef __DRIVER_CLOUD_H
#define __DRIVER_CLOUD_H

#include "stm32f4xx.h"

#define Pitch_MaxAngle 30.0f
#define Pitch_MinAngle -25.0f

typedef struct {
  float AnglePosition;
  float TargetAngle;
  float AngularSpeed;
} Cloud_Type;

void CloudPara_Init(Cloud_Type *CourseAngle);

void TargetAngleSet(float pitch_control, float yaw_control);

#endif
