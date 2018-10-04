
#ifndef __DRIVER_GYROSCOPE_H
#define __DRIVER_GYROSCOPE_H

#include "stm32f4xx.h"
#include "math.h"
#include "mpu6500_driver.h"
#include "stdio.h"

#define Q_SIZE 4
#define E_size 3
#define PI 3.1415926
#define GYRO_LSB 16.4f
#define ACC_LSB 4096.0

// e_index是欧拉角索引,x轴向机头,y轴向机翼,yaw为z轴旋转,pitch为y轴旋转
// q_index是四元数索引,mag是实部
typedef enum { yaw, pitch, roll } i_euler;
typedef enum { mag, ix, iy, iz } i_qud;

typedef struct {
    float Yaw;
    int   Yaw_offset;
    int   round;
    float Yawfeedback;
    float Pitch;
    int   Pitch_offset;
    float Roll;
} EulerAngle_Type;

typedef struct {
    double rawValue;
    double xbuf[8];
    double ybuf[8];
    double filtered_value;
} Filter_t;

#ifdef __DRIVER_GYROSCOPE_GLOBALS
#define __DRIVER_GYROSCOPE_EXT
#else
#define __DRIVER_GYROSCOPE_EXT extern
#endif

__DRIVER_GYROSCOPE_EXT volatile EulerAngle_Type EulerAngle;
extern volatile float                           angle[3];

void Gyroscope_Update_Angle_Data(void);

#endif
