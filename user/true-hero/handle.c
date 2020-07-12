/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

void Handle_Init(void) {
    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

    // 发射机构电机
    Motor_Init(&Motor_LeftFrict, 1, DISABLE, ENABLE);
    Motor_Init(&Motor_RightFrict, 1, DISABLE, ENABLE);
    Motor_Init(&Motor_Stir3510, 19.2, DISABLE, ENABLE);

    // 云台电机
    Motor_Init(&Motor_Yaw, 1, ENABLE, ENABLE);
    Motor_Init(&Motor_Pitch, 1, ENABLE, ENABLE);

    // CAN外设
    Can1_Device[ESC_ID(0x203)] = &Motor_Yaw;
    Can1_Device[ESC_ID(0x204)] = &Motor_Stir3510;
    Can1_Device[ESC_ID(0x205)] = &Motor_LF;
    Can1_Device[ESC_ID(0x206)] = &Motor_LB;
    Can1_Device[ESC_ID(0x207)] = &Motor_RB;
    Can1_Device[ESC_ID(0x208)] = &Motor_RF;

    Can2_Device[ESC_ID(0x201)] = &Motor_LeftFrict;
    Can2_Device[ESC_ID(0x202)] = &Motor_RightFrict;
    Can2_Device[ESC_ID(0x206)] = &Motor_Pitch;

    // 陀螺仪设置静态误差
    Gyroscope_Set_Bias(&ImuData, 0.15, 34.65, -8.55);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);
}
