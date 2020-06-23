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
    Protocol_Init(&Judge);
    Protocol_Init(&Ps);
}
