#ifndef __MPU6500_DRIVER_H__
#define __MPU6500_DRIVER_H__
#include "stm32f4xx.h"
#include "delay.h"
#include "mpu6500_i2c.h"
#include "mpu6500_IST8310.h"

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
} IMU_DataType;

#ifdef __DRIVER_MPU6500_GLOBALS
#define __DRIVER_MPU6500_EXT
#else
#define __DRIVER_MPU6500_EXT extern
#endif

__DRIVER_MPU6500_EXT IMU_DataType mpu6500_data;
__DRIVER_MPU6500_EXT int16_t IMU_id;
__DRIVER_MPU6500_EXT uint8_t mpu_buf[20];

int MPU6500_Init(void);

void MPU6500_Initialize(void);

int MPU6500_ReadData(uint8_t Slave_Addr, uint8_t Reg_Addr, uint8_t *Data, uint8_t Num);
int MPU6500_EnableInt(void);

void MPU6500_getMotion6(void);

#endif
