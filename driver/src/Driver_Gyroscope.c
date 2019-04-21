#define __DRIVER_GYROSCOPE_GLOBALS
#include "math.h"
#include "Driver_Filter.h"
#include "config.h"
#include "MadgwickAHRS.h"
#include "Driver_Gyroscope.h"

static float rollAngle;
static float pitchAngle;
static float yawAngle;
static float xSpeed;
static float ySpeed;
static float zSpeed;
static float xAcc;
static float yAcc;
static float zAcc;

int debugA = 0;
int debugB = 0;
int debugC = 0;
int debugD = 0;
int debugE = 0;
int debugF = 0;

Filter_Type Filter_Yaw = {.count = 0, .thresholdLB = GYROSCOPE_YAW_FILTER_THRESHOLD};

extern ImuData_Type       ImuData;
extern GyroscopeData_Type Gyroscope_EulerData;

int test = 0;

void Gyroscope_Init(GyroscopeData_Type *GyroscopeData) {
    GyroscopeData->startupCounter = 0;
    MPU6500_Initialize();
    MPU6500_EnableInt();
#if GYROSCOPE_START_UP_DELAY_ENABLED
    while (1) {
        if (Gyroscope_EulerData.startupCounter >= GYROSCOPE_START_UP_DELAY) break;
        test = test % 1000 + 1;
    }
#endif
}

void Gyroscope_Update_Angle_Data(GyroscopeData_Type *GyroscopeData) {
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
    // todo:验证有效性
    if (GyroscopeData->startupCounter == GYROSCOPE_START_UP_DELAY - 1) {
        GyroscopeData->yawoffset = -GyroscopeData->yaw;
    }

    GyroscopeData->pitch = -pitchAngle; // todo:这负号干嘛的?
    GyroscopeData->roll  = rollAngle;

    // todo:删掉debugAB
    debugA = GyroscopeData->yaw;
    debugB = GyroscopeData->pitch;
    debugC = GyroscopeData->roll;
    debugD = ImuData.gx;
    debugE = ImuData.gy;
    debugF = ImuData.gz;

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
