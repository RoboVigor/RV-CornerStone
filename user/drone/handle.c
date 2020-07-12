/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

void Handle_Init(void) {
    //电机初始化
    // Motor_Init(&Motor_Yaw, 1.0, 1);
    // Motor_Init(&Motor_Pitch, 1.0, 1);
    // Motor_Init(&Motor_Roll, 1.0, 1);
    // Motor_Init(&Motor_Stir, 36, 1);
    Motor_Init(&Motor_Yaw, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_Pitch, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_Roll, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_Stir, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Pitch.position     = 2740;
    Motor_Pitch.positionBias = 2740;
    Motor_Yaw.position       = 3538;
    Motor_Yaw.positionBias   = 3538;

    // CAN外设
    Can1_Device[ESC_ID(0x201)] = &Motor_Yaw;
    Can1_Device[ESC_ID(0x202)] = &Motor_Pitch;
    Can1_Device[ESC_ID(0x203)] = &Motor_Roll;
    Can1_Device[ESC_ID(0x204)] = &Motor_Stir;

    // 陀螺仪设置静态误差
    Gyroscope_Set_Bias(&ImuData, 0.69, -8, -7.6);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);
}
