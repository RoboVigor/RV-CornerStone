#ifndef __MPU6500_DRIVER_H__
#define __MPU6500_DRIVER_H__
#include "delay.h"
#include "mpu6500_IST8310.h"
#include "mpu6500_i2c.h"
#include "stm32f4xx.h"

typedef struct {
    volatile int16_t ax; // m/s^2 [-8g,+8g] -> [-32768,32768] ideal:0
    volatile int16_t ay; // m/s^2 [-8g,+8g] -> [-32768,32768] ideal:0
    volatile int16_t az; // m/s^2 [-8g,+8g] -> [-32768,32768] ideal:4096
    volatile int16_t temp;
    volatile int16_t gx; // rad/s ideal:0
    volatile int16_t gy; // rad/s ideal:0
    volatile int16_t gz; // rad/s ideal:0
    int16_t          ax_offset;
    int16_t          ay_offset;
    int16_t          az_offset;
    int16_t          gx_offset;
    int16_t          gy_offset;
    int16_t          gz_offset;
} ImuData_Type;

#ifdef __DRIVER_MPU6500_GLOBALS
#define __DRIVER_MPU6500_EXT
#else
#define __DRIVER_MPU6500_EXT extern
#endif

int MPU6500_Init(void);

void MPU6500_Initialize(void);

int MPU6500_ReadData(void);
int MPU6500_EnableInt(void);

void MPU6500_getMotion6(void);

#endif
