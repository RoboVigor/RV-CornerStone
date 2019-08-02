/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"

void Handle_Init(void) {
    // 底盘电机
    Motor_Init(&Motor_Chassis_Left, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_Chassis_Right, CHASSIS_MOTOR_REDUCTION_RATE, 0);

    // 云台电机
    Motor_Init(&Motor_Stabilizer_Yaw, 1, 1);
    Motor_Init(&Motor_Stabilizer_Pitch, 36, 1);

    Motor_Stabilizer_Yaw.positionBias = 1378;
    Motor_Stabilizer_Yaw.position     = 1378;

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&Judge);
    Protocol_Init(&Ps);
}
