/**
 * @brief 用于解算欧拉角和陀螺仪零飘滤波
 * @note g和a是gyroscope和acceleration的缩写
 */

#define __DRIVER_ANGULAR_GLOBALS
#include "Driver_Angular.h"
#include "MadgwickAHRS.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"

float e_angle[3] = {0, 0, 0}; // yaw,pitch,roll
float a_speed[4] = {0, 0, 0, 0};

float ax_acc = 0;
float ay_acc = 0;
float az_acc = 0;

void Gyroscope_Update_Angle_Data(void) {
    a_speed[1] = (float) ((mpu6500_data.gx / GYRO_LSB) * PI / 180);
    a_speed[2] = (float) ((mpu6500_data.gy / GYRO_LSB) * PI / 180);
    a_speed[3] = (float) ((mpu6500_data.gz / GYRO_LSB) * PI / 180);
    ax_acc     = (float) (mpu6500_data.ax / ACC_LSB);
    ay_acc     = (float) (mpu6500_data.ay / ACC_LSB);
    az_acc     = (float) (mpu6500_data.az / ACC_LSB);

    // GD算法或Madgwick算法,梯度算法,网上开源
    MadgwickAHRSupdateIMU(a_speed[1], a_speed[2], a_speed[3], ax_acc, ay_acc, az_acc);

    e_angle[1] = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180 / PI;
    e_angle[0] = asin(2.0f * (q0 * q2 - q1 * q3)) * 180 / PI;
    e_angle[2] = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180 / PI;

    if (e_angle[1] >= 0) {
        e_angle[1] -= 180.0;
    } else {
        e_angle[1] += 180.0;
    }

    EulerAngle.Yaw   = e_angle[2];
    EulerAngle.Pitch = -e_angle[1];
    EulerAngle.Pitch += EulerAngle.Pitch_offset;
    EulerAngle.Roll = e_angle[0];
}