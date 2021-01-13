/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

void Handle_Init(void) {
    // 底盘电机
    Motor_Init(&Motor_Windmill, CHASSIS_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);

    // CAN外设
    Can1_Device[ESC_ID(0x201)] = &Motor_Windmill;

    // 陀螺仪设置静态误差
    // Gyroscope_Set_Bias(&ImuData, -4, 3, 1);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 通讯协议初始化
    // Protocol_Init(&JudgeChannel, &ProtocolData);
    // Protocol_Init(&HostChannel, &ProtocolData);
    // Protocol_Init(&UserChannel, &ProtocolData);
}
