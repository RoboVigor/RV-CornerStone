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
#ifdef STM32F427_437xx
#include "mpu6500_driver.h"
#endif
#ifdef STM32F40_41xxx
#include "BMI088driver.h"
#include "ist8310driver.h"
#endif

#define PI 3.1415926f

typedef struct {
    volatile int16_t ax; // m/s^2 [-8g,+8g] -> [-32768,32768] ideal:0
    volatile int16_t ay; // m/s^2 [-8g,+8g] -> [-32768,32768] ideal:0
    volatile int16_t az; // m/s^2 [-8g,+8g] -> [-32768,32768] ideal:4096
    volatile int16_t temp;
    volatile int16_t gx; // rad/s ideal:0
    volatile int16_t gy; // rad/s ideal:0
    volatile int16_t gz; // rad/s ideal:0
    volatile int16_t mx; // rad/s ideal:0
    volatile int16_t my; // rad/s ideal:0
    volatile int16_t mz; // rad/s ideal:0
    int16_t          ax_offset;
    int16_t          ay_offset;
    int16_t          az_offset;
    int16_t          gx_offset;
    int16_t          gy_offset;
    int16_t          gz_offset;
    int16_t          mx_offset;
    int16_t          my_offset;
    int16_t          mz_offset;
    int16_t          gx_bias;
    int16_t          gy_bias;
    int16_t          gz_bias;
} ImuData_Type;
typedef struct {
    float yaw;
    float pitch;
    float roll;
    float yawoffset;
    int   startupCounter;
    float modification;
} GyroscopeData_Type;

/**
 * @brief 陀螺仪初始化
 * @param startupDelay: 等待初始化解算, 0为关闭, 通常取150-300
 */
void Gyroscope_Init(GyroscopeData_Type *GyroscopeData, uint16_t startupDelay);

/**
 * @brief 陀螺仪更新
 */
Gyroscope_Update(GyroscopeData_Type *GyroscopeData);

/**
 * @brief 欧拉角解算
 * @note  由Gyroscope_Update()调用,不需要在中断中使用
 */
void Gyroscope_Solve(GyroscopeData_Type *GyroscopeData);

/**
 * @brief 获得滤波器diff
 */
float Gyroscope_Get_Filter_Diff(void);

/**
 * @brief 设置静态误差
 */
void Gyroscope_Set_Bias(ImuData_Type *ImuData, int16_t gx_bias, int16_t gy_bias, int16_t gz_bias);

#endif
