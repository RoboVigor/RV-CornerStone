#define __DRIVER_GYROSCOPE_GLOBALS
#include "Driver_Gyroscope.h"
#include "Driver_Filter.h"
#include "MadgwickAHRS.h"
#include "config.h"
#include "handle.h"
#include "mpu6500_driver.h"

float e_angle[3] = {0, 0, 0}; // yaw,pitch,roll
float a_speed[4] = {0, 0, 0, 0};

float ax_acc = 0;
float ay_acc = 0;
float az_acc = 0;

Filter_Type Filter_Yaw = {.count = 0,
                          .thresholdLB = GYROSCOPE_YAW_FILTER_THRESHOLD};

void Gyroscope_Update_Angle_Data(void) {
  a_speed[1] = (float)((mpu6500_data.gx / GYRO_LSB) * PI / 180);
  a_speed[2] = (float)((mpu6500_data.gy / GYRO_LSB) * PI / 180);
  a_speed[3] = (float)((mpu6500_data.gz / GYRO_LSB) * PI / 180);
  ax_acc = (float)(mpu6500_data.ax / ACC_LSB);
  ay_acc = (float)(mpu6500_data.ay / ACC_LSB);
  az_acc = (float)(mpu6500_data.az / ACC_LSB);

  // GD算法或Madgwick算法,梯度算法,网上开源
  MadgwickAHRSupdateIMU(a_speed[1], a_speed[2], a_speed[3], ax_acc, ay_acc,
                        az_acc);

  e_angle[1] =
      atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) *
      180 / PI;
  e_angle[0] = asin(2.0f * (q0 * q2 - q1 * q3)) * 180 / PI;
  e_angle[2] =
      atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
      180 / PI;

  if (e_angle[1] >= 0) {
    e_angle[1] -= 180.0;
  } else {
    e_angle[1] += 180.0;
  }

  // 更新滤波器
  Filter_Update(&Filter_Yaw, e_angle[2]);

  // 计算连续 Yaw 角
  if (Filter_Yaw.diff > 300) {
    Filter_Yaw.offset -= 360;
  } else if (Filter_Yaw.diff < -300) {
    Filter_Yaw.offset += 360;
  }

  // 输出欧拉角
  EulerAngle.Yaw = Filter_Apply_Limit_Breadth(&Filter_Yaw); // 应用限幅滤波
  EulerAngle.Pitch = -e_angle[1];
  EulerAngle.Pitch += EulerAngle.Pitch_offset;
  EulerAngle.Roll = e_angle[0];
}

float Gyroscope_Get_Filter_Diff(void) { return Filter_Yaw.diff; }
