/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"

void Handle_Init(void) {
    // Chasis
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, 0);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, 0);

    // 取弹左右平移
    Motor_Init(&Motor_TH, 36, 1); // 2006
    Motor_Init(&Motor_TV, 19.2f, 1); // 3510

    // Guide wheel
    Motor_Init(&Motor_LGW, 36, 1);
    Motor_Init(&Motor_RGW, 36, 1);

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);
}
