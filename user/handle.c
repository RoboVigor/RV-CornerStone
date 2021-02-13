/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"

void Handle_Init(void) {
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_SumsungLB, CHASSIS_MOTOR_REDUCTION_RATE, 1);
    Motor_Init(&Motor_SumsungRB, CHASSIS_MOTOR_REDUCTION_RATE, 1);
    Motor_Init(&Motor_SumsungRF, CHASSIS_MOTOR_REDUCTION_RATE, 1);
    Motor_Init(&Motor_SumsungLF, CHASSIS_MOTOR_REDUCTION_RATE, 1);
}