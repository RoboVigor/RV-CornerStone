/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"

void Handle_Init(void) {
  Motor_Init(&Motor_Chassis_Left, CHASSIS_MOTOR_REDUCTION_RATE, 0);
  Motor_Init(&Motor_Chassis_Right, CHASSIS_MOTOR_REDUCTION_RATE, 0);
  Motor_Init(&Motor_Stabilizer_Yaw, CHASSIS_MOTOR_REDUCTION_RATE, 1);
  Motor_Init(&Motor_Stabilizer_Pitch, CHASSIS_MOTOR_REDUCTION_RATE, 1);
  Gyroscope_EulerData.downcounter = 0;
}
