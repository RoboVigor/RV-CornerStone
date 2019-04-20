/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"

void Handle_Init(void) {
    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, 0);

    // 发射机构电机
    Motor_Init(&Motor_LeftFrict, 1, 0);
    Motor_Init(&Motor_RightFrict, 1, 0);
    Motor_Init(&Motor_Stir, 36, 1);

    // 云台电机
    Motor_Pitch.positionBias = 1300;
    Motor_Init(&Motor_Yaw, 1.0, 1);
    Motor_Init(&Motor_Pitch, 1.0, 1);

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);
}
