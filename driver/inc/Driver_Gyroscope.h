/**
 * @file Driver_Gyroscope.h
 * @brief 陀螺仪驱动,用于解算欧拉角和陀螺仪零飘滤波
 * @note g和a是gyroscope和acceleration的缩写
 * @version 0.6
 * - 精简无用代码
 * - 应用Filter_LB
 */

#ifndef __DRIVER_GYROSCOPE_H
#define __DRIVER_GYROSCOPE_H

#include "stm32f4xx.h"

#define PI 3.1415926f
#define GYRO_LSB 16.4f
#define ACC_LSB 4096.0f

typedef struct {
    float yaw;
    float pitch;
    float roll;
    float yawoffset;
    int   downcounter;
} GyrosocopeData_Type;

/**
 * @brief 欧拉角解算
 */
void Gyroscope_Update_Angle_Data(GyrosocopeData_Type *GyrosocopeData);

/**
 * @brief 获得滤波器diff
 */
float Gyroscope_Get_Filter_Diff(void);

#endif
