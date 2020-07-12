/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "marco.h"

void Handle_Init(void) {
    // 电机
    Motor_Init(&Motor_Charge, 1, DISABLE,ENABLE);
    Can1_Device[ESC_ID(0x201)] = &Motor_Charge;
    

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);
}
