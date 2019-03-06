#define __DRIVER_GYROSCOPE_GLOBALS
#include "Driver_Gyroscope.h"
#include "Driver_Filter.h"
#include "config.h"
#include "handle.h"
#include "MadgwickAHRS.h"

static float rollAngle;
static float pitchAngle;
static float yawAngle;
static float xSpeed;
static float ySpeed;
static float zSpeed;
static float xAcc;
static float yAcc;
static float zAcc;

Filter_Type Filter_Yaw = {.count = 0, .thresholdLB = GYROSCOPE_YAW_FILTER_THRESHOLD};

void Gyroscope_Update_Angle_Data(void) {

    xSpeed = (float) ((mpu6500_data.gx / GYRO_LSB) * PI / 180.0);
    ySpeed = (float) ((mpu6500_data.gy / GYRO_LSB) * PI / 180.0);
    zSpeed = (float) ((mpu6500_data.gz / GYRO_LSB) * PI / 180.0);
    xAcc   = (float) (mpu6500_data.ax / ACC_LSB);
    yAcc   = (float) (mpu6500_data.ay / ACC_LSB);
    zAcc   = (float) (mpu6500_data.az / ACC_LSB);

    // GD算法或Madgwick算法,梯度算法,网上开源
    MadgwickAHRSupdateIMU(xSpeed, ySpeed, zSpeed, xAcc, yAcc, zAcc);

    pitchAngle = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180 / PI;
    rollAngle  = asin(2.0f * (q0 * q2 - q1 * q3)) * 180 / PI;
    yawAngle   = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180 / PI;

    // if (pitchAngle >= 0) {
    //    pitchAngle -= 180.0;
    //} else {
    //    pitchAngle += 180.0;
    //}

    // 更新滤波器
    Filter_Update(&Filter_Yaw, yawAngle);

    // 计算连续 Yaw 角
    if (Filter_Yaw.diff > 300) {
        Filter_Yaw.offset -= 360;
    } else if (Filter_Yaw.diff < -300) {
        Filter_Yaw.offset += 360;
    }

    // 输出欧拉角

    Gyroscope_EulerData.yaw = Filter_Apply_Limit_Breadth(&Filter_Yaw) + Gyroscope_EulerData.yawoffset; // 应用限幅滤波
    if (Gyroscope_EulerData.downcounter == GYROSCOPE_START_UP_DELAY - 1) {
        Gyroscope_EulerData.yawoffset = -Gyroscope_EulerData.yaw;
    }

    Gyroscope_EulerData.pitch = -pitchAngle;
    Gyroscope_EulerData.roll  = rollAngle;
}

float Gyroscope_Get_Filter_Diff(void) {
    return Filter_Yaw.diff;
}
