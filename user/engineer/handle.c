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
    Motor_Init(&Motor_TakeLeft, CHASSIS_MOTOR_REDUCTION_RATE, 1);
    Motor_Init(&Motor_TakeRight, CHASSIS_MOTOR_REDUCTION_RATE, 1);
    Motor_Init(&Motor_Transmission, 36, 1);
    Motor_Init(&Motor_Upthrow, 1, 1);
    Motor_Init(&Motor_LGW, 1, 1);
    Motor_Init(&Motor_RGW, 1, 1);
}
