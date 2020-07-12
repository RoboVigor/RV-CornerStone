/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

void Handle_Init(void) {
    // 底盘电机
    Motor_Init(&Motor_Chassis_Left, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_Chassis_Right, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

    // 云台电机
    Motor_Init(&Motor_Stabilizer_Yaw, 1, ENABLE, DISABLE);
    Motor_Init(&Motor_Stabilizer_Pitch, 36, ENABLE, DISABLE);

    // 摩擦轮电机
    Motor_Init(&Motor_Frict_L, 1, DISABLE, ENABLE);
    Motor_Init(&Motor_Frict_R, 1, DISABLE, ENABLE);

    // 发射机构电机
    Motor_Init(&Motor_Stir, 1, DISABLE, DISABLE);

    Motor_Stabilizer_Yaw.positionBias = 1378;
    Motor_Stabilizer_Yaw.position     = 1378;

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // CAN外设
    Can1_Device[ESC_ID(0x201)] = &Motor_Chassis_Left;
    Can1_Device[ESC_ID(0x202)] = &Motor_Chassis_Right;
    Can1_Device[ESC_ID(0x203)] = &Motor_Frict_L;
    Can1_Device[ESC_ID(0x204)] = &Motor_Frict_R;
    Can1_Device[ESC_ID(0x207)] = &Motor_Stir;

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);
}
