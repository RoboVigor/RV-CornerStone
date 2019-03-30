/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"

void Handle_Init(void) {
  Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, 0);
  Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, 0);
  Gyroscope_EulerData.downcounter = 0;
}
