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

    // 云台电机
    Motor_Init(&Motor_Yaw, 1.0, 1);   // 顺时针为正电流
    Motor_Init(&Motor_Pitch, 1.0, 1); // 逆时针为正电流
    Motor_Init(&Motor_Stir, 1.0, 1);  //拨弹
    Motor_Yaw.positionBias   = 1354;
    Motor_Yaw.position       = 1354;
    Motor_Pitch.positionBias = 4766;
    Motor_Pitch.position     = 4766;

    //抬升电机
    Motor_Init(&Motor_Raise_Left, 19.2f, 1);
    Motor_Init(&Motor_Raise_Right, 19.2f, 1);

    // 抓取电机
    Motor_Init(&Motor_Fetch_X, FITCH_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_Fetch_Left_Pitch, FITCH_MOTOR_REDUCTION_RATE, 1);
    Motor_Init(&Motor_Fetch_Right_Pitch, FITCH_MOTOR_REDUCTION_RATE, 1);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);
}
