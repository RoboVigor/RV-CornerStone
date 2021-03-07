/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

void Handle_Init(void) {
    // 底盘电机
    Motor_Init(&Motor_Chassis_Left, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);
    Motor_Init(&Motor_Chassis_Right, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);

    // 云台电机
    // 上
    Motor_Init(&Motor_Up_Gimbal_Yaw, 36, ENABLE, ENABLE);
    Motor_Init(&Motor_Up_Gimbal_Pitch, 36, ENABLE, ENABLE);
    // 下
    Motor_Init(&Motor_Down_Gimbal_Yaw, 1, DISABLE, DISABLE);
    Motor_Init(&Motor_Down_Gimbal_Pitch, 1, DISABLE, DISABLE);

    // 摩擦轮电机
    // 上
    Motor_Init(&Motor_Up_Frict_Left, 1, DISABLE, DISABLE);
    Motor_Init(&Motor_Up_Frict_Right, 1, DISABLE, DISABLE);
    // 下
    Motor_Init(&Motor_Down_Frict_Left, 1, DISABLE, DISABLE);
    Motor_Init(&Motor_Down_Frict_Right, 1, DISABLE, DISABLE);

    // 拨弹电机
    // 上
    Motor_Init(&Motor_Up_Stir, 1, DISABLE, DISABLE);
    // 下
    Motor_Init(&Motor_Down_Stir, 1, DISABLE, DISABLE);

    // Motor_Up_Gimbal_Pitch.positionBias   = ;
    // Motor_Up_Gimbal_Pitch.position       = ;
    // Motor_Up_Gimbal_Yaw.positionBias     = ;
    // Motor_Up_Gimbal_Yaw.position         = ;
    // Motor_Down_Gimbal_Pitch.positionBias = ;
    // Motor_Down_Gimbal_Pitch.position     = ;
    // Motor_Down_Gimbal_Yaw.positionBias   = ;
    // Motor_Down_Gimbal_Yaw.position       = ;

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // CAN外设
    Can1_Device[ESC_ID(0x201)] = &Motor_Chassis_Left;
    Can1_Device[ESC_ID(0x202)] = &Motor_Chassis_Right;

    Can1_Device[ESC_ID(0x203)] = &Motor_Down_Frict_Left;
    Can1_Device[ESC_ID(0x204)] = &Motor_Down_Frict_Right;
    Can1_Device[ESC_ID(0x205)] = &Motor_Down_Stir;
    Can2_Device[ESC_ID(0x207)] = &Motor_Down_Gimbal_Yaw;
    Can1_Device[ESC_ID(0x209)] = &Motor_Down_Gimbal_Pitch;

    Can2_Device[ESC_ID(0x203)] = &Motor_Up_Frict_Left;
    Can2_Device[ESC_ID(0x204)] = &Motor_Up_Frict_Right;
    Can2_Device[ESC_ID(0x205)] = &Motor_Up_Stir;
    Can1_Device[ESC_ID(0x208)] = &Motor_Up_Gimbal_Yaw;
    Can1_Device[ESC_ID(0x206)] = &Motor_Up_Gimbal_Pitch;

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);
}
