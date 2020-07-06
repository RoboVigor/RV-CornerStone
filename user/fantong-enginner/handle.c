/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"
#include "macro.h"

void Handle_Init(void) {
#ifdef BOARD_ALPHA
    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

    // 云台电机
    Motor_Init(&Motor_Yaw, 1.0, ENABLE, ENABLE);    // 顺时针为正电流
    Motor_Init(&Motor_Pitch, 1.0, ENABLE, DISABLE); // 逆时针为正电流
    Motor_Init(&Motor_Stir, 1.0, ENABLE, ENABLE);   //拨弹
    Motor_Yaw.positionBias   = 1354;
    Motor_Yaw.position       = 1354;
    Motor_Pitch.positionBias = 4766;
    Motor_Pitch.position     = 4766;

    //抬升电机
    Motor_Init(&Motor_Raise_Left, 19.2f, ENABLE, ENABLE);
    Motor_Init(&Motor_Raise_Right, 19.2f, ENABLE, ENABLE);

    // CAN外设
    Can1_Device[ESC_ID(0x201)] = &Motor_LF;
    Can1_Device[ESC_ID(0x202)] = &Motor_LB;
    Can1_Device[ESC_ID(0x203)] = &Motor_RB;
    Can1_Device[ESC_ID(0x204)] = &Motor_RF;
    Can1_Device[ESC_ID(0x206)] = &Motor_Pitch;
    Can1_Device[ESC_ID(0x207)] = &Motor_Raise_Left;
    Can1_Device[ESC_ID(0x208)] = &Motor_Raise_Right;
    Can1_Device[ESC_ID(0x209)] = &Motor_Yaw;
    Can2_Device[ESC_ID(0x207)] = &Motor_Stir;

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 通讯协议初始化
    Protocol_Init(&JudgeChannel, &ProtocolData);
    Protocol_Init(&HostChannel, &ProtocolData);
    Protocol_Init(&UserChannel, &ProtocolData);

    // 陀螺仪设置静态误差
    Gyroscope_Set_Bias(&ImuData, 28, 30, 0);

#endif
#ifdef BOARD_BETA

    // 抓取电机
    Motor_Init(&Motor_Fetch_X, FITCH_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_Fetch_Left_Pitch, FITCH_MOTOR_REDUCTION_RATE, 1);
    Motor_Init(&Motor_Fetch_Right_Pitch, FITCH_MOTOR_REDUCTION_RATE, 1);

#endif
}
