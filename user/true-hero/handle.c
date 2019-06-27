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
    Motor_Init(&Motor_Stir2006, 1, 0);
    Motor_Init(&Motor_Stir3510, 1, 0);

    // 云台电机
    Motor_Init(&Motor_Yaw, 1.0, 1);   // 顺时针为正电流
    Motor_Init(&Motor_Pitch, 1.0, 1); // 逆时针为正电流
    Motor_Yaw.positionBias = 5070;
    Motor_Yaw.position     = 5070;

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&Judge);
    Protocol_Init(&Ps);
}
