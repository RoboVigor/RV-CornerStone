#define __DRIVER_GYROSCOPE_GLOBALS
#include "math.h"
#include "Driver_Filter.h"
#include "config.h"
#include "MadgwickAHRS.h"
#include "Driver_Gyroscope.h"

static float          rollAngle;
static float          pitchAngle;
static float          yawAngle;
static float          xSpeed;
static float          ySpeed;
static float          zSpeed;
static float          xAcc;
static float          yAcc;
static float          zAcc;
extern volatile float beta;
static int16_t        debug_pitch = 0;

Filter_Type Filter_Yaw = {.count = 0, .thresholdLB = GYROSCOPE_YAW_FILTER_THRESHOLD};

extern ImuData_Type ImuData;

void Gyroscope_Init(GyroscopeData_Type *GyroscopeData) {
    GyroscopeData->startupCounter = 0;
    MPU6500_Initialize();
    MPU6500_EnableInt();
#if GYROSCOPE_START_UP_DELAY_ENABLED
    beta = 5;
    while (1) {
        LED_Set_Progress(GyroscopeData->startupCounter / (GYROSCOPE_START_UP_DELAY / 7) + 1);
        if (GyroscopeData->startupCounter >= GYROSCOPE_START_UP_DELAY) {
            beta = 0.1;
            break;
        }
    }
#endif
}

// MPU6500数据读取,成功返回1  失败返回0
int Gyroscope_Update(GyroscopeData_Type *GyroscopeData) {
    static uint8_t mpu_buf[20];

    //尝试读取数据
    if (IIC_ReadData(MPU_IIC_ADDR, MPU6500_ACCEL_XOUT_H, mpu_buf, 14) == 0xff) return 0;

    //成功的话进行赋值
    ImuData.temp = (((int16_t) mpu_buf[6]) << 8) | mpu_buf[7];
#if BOARD_FRONT_IS_UP
    ImuData.az = (((int16_t) mpu_buf[4]) << 8) | mpu_buf[5];
    ImuData.gz = ((((int16_t) mpu_buf[12]) << 8) | mpu_buf[13]) - (IMU_GZ_BIAS);
#else
    ImuData.az = -1 * (((int16_t) mpu_buf[4]) << 8) | mpu_buf[5];
    ImuData.gz = -1 * ((((int16_t) mpu_buf[12]) << 8) | mpu_buf[13]) - (IMU_GZ_BIAS);
#endif
#if BOARD_SHORT_SIDE_IS_PARALLEL_TO_PITCH
    ImuData.ax = (((int16_t) mpu_buf[0]) << 8) | mpu_buf[1];
    ImuData.ay = (((int16_t) mpu_buf[2]) << 8) | mpu_buf[3];
    ImuData.gx = ((((int16_t) mpu_buf[8]) << 8) | mpu_buf[9]) - (IMU_GX_BIAS);
    ImuData.gy = ((((int16_t) mpu_buf[10]) << 8) | mpu_buf[11]) - (IMU_GY_BIAS);
#else
    ImuData.ay = (((int16_t) mpu_buf[0]) << 8) | mpu_buf[1];
    ImuData.ax = (((int16_t) mpu_buf[2]) << 8) | mpu_buf[3];
    ImuData.gy = ((((int16_t) mpu_buf[8]) << 8) | mpu_buf[9]) - (IMU_GY_BIAS);
    ImuData.gx = ((((int16_t) mpu_buf[10]) << 8) | mpu_buf[11]) - (IMU_GX_BIAS);
#endif

    // 读取完成进行解算
    Gyroscope_Solve(GyroscopeData);

    // 返回成功值
    return 1;
}

void Gyroscope_Solve(GyroscopeData_Type *GyroscopeData) {
    xSpeed = (float) ((ImuData.gx / GYRO_LSB) * PI / 180.0);
    ySpeed = (float) ((ImuData.gy / GYRO_LSB) * PI / 180.0);
    zSpeed = (float) ((ImuData.gz / GYRO_LSB) * PI / 180.0);
    xAcc   = (float) (ImuData.ax / ACC_LSB);
    yAcc   = (float) (ImuData.ay / ACC_LSB);
    zAcc   = (float) (ImuData.az / ACC_LSB);

    // GD算法或Madgwick算法,梯度算法,网上开源
    MadgwickAHRSupdateIMU(xSpeed, ySpeed, zSpeed, xAcc, yAcc, zAcc);

    // 四元数->欧拉角
    pitchAngle = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180 / PI;
    rollAngle  = asin(2.0f * (q0 * q2 - q1 * q3)) * 180 / PI;
    yawAngle   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180 / PI;

    // 更新滤波器
    Filter_Update(&Filter_Yaw, yawAngle);

    // 计算连续 Yaw 角
    if (Filter_Yaw.diff > 300) {
        Filter_Yaw.offset -= 360;
    } else if (Filter_Yaw.diff < -300) {
        Filter_Yaw.offset += 360;
    }

    // 应用滤波
    GyroscopeData->yaw = Filter_Apply_Limit_Breadth(&Filter_Yaw) + GyroscopeData->yawoffset;

    // 输出欧拉角
    if (GyroscopeData->startupCounter == GYROSCOPE_START_UP_DELAY - 1) {
        GyroscopeData->yawoffset = -GyroscopeData->yaw;
    }

    GyroscopeData->pitch = -pitchAngle;
    GyroscopeData->roll  = rollAngle;
    debug_pitch          = GyroscopeData->pitch;

    // 输出欧拉角
#if GYROSCOPE_START_UP_DELAY_ENABLED
    if (GyroscopeData->startupCounter < GYROSCOPE_START_UP_DELAY) {
        GyroscopeData->startupCounter += 1;
    }
#endif
}

float Gyroscope_Get_Filter_Diff(void) {
    return Filter_Yaw.diff;
}
