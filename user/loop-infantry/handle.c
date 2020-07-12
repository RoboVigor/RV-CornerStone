/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

void Handle_Init(void) {

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);

    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

    // 发射机构电机
    Motor_Init(&Motor_Stir, STIR_MOTOR_REDUCTION_RATE, ENABLE, ENABLE); //拨弹
    Motor_Init(&Motor_FL, FIRE_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_FR, FIRE_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

    // 云台电机
    Motor_Init(&Motor_Yaw, GIMBAL_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);   // 顺时针为正电流
    Motor_Init(&Motor_Pitch, GIMBAL_MOTOR_REDUCTION_RATE, ENABLE, ENABLE); // 顺时针为正电流

#ifdef ROBOT_LOOP_ONE
    Motor_Yaw.positionBias   = 4110;
    Motor_Yaw.position       = 4110;
    Motor_Pitch.positionBias = 5540;
    Motor_Pitch.position     = 5540;
#endif
#ifdef ROBOT_LOOP_TWO
    Motor_Yaw.positionBias   = 3400;
    Motor_Yaw.position       = 3400;
    Motor_Pitch.positionBias = 7090;
    Motor_Pitch.position     = 7090;
#endif

    // CAN外设
    Can1_Device[ESC_ID(0x201)] = &Motor_LF;
    Can1_Device[ESC_ID(0x202)] = &Motor_LB;
    Can1_Device[ESC_ID(0x203)] = &Motor_RB;
    Can1_Device[ESC_ID(0x204)] = &Motor_RF;
    Can1_Device[ESC_ID(0x206)] = &Motor_Pitch;
    Can1_Device[ESC_ID(0x209)] = &Motor_Yaw;
    Can2_Device[ESC_ID(0x201)] = &Motor_FL;
    Can2_Device[ESC_ID(0x202)] = &Motor_FR;
    Can2_Device[ESC_ID(0x207)] = &Motor_Stir;

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);
}
