/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

void Handle_Init(void) {
    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);

    // CAN外设
    Can1_Device[ESC_ID(0x201)] = &Motor_LF;
    Can1_Device[ESC_ID(0x202)] = &Motor_LB;
    Can1_Device[ESC_ID(0x203)] = &Motor_RB;
    Can1_Device[ESC_ID(0x204)] = &Motor_RF;

    // 陀螺仪设置静态误差
    Gyroscope_Set_Bias(&ImuData, 2, 9, -6);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);
}
