#ifndef __MPU6500_DRIVER_H__
#define __MPU6500_DRIVER_H__
#include "delay.h"
#include "mpu6500_IST8310.h"
#include "mpu6500_i2c.h"
#include "stm32f4xx.h"

typedef struct {
    volatile int16_t ax;
    volatile int16_t ay;
    volatile int16_t az;
    volatile int16_t temp;
    volatile int16_t gx;
    volatile int16_t gy;
    volatile int16_t gz;
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
