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

    // 取弹前后左右平移
    Motor_Init(&Motor_TH, 36, 1);    // 2006
    Motor_Init(&Motor_TV, 19.2f, 1); // 3510

    // 抬升
    Motor_Init(&Motor_Upthrow1, 19.2f, 1);
    Motor_Init(&Motor_Upthrow2, 19.2f, 1);

    // 翻转
    Motor_Init(&Motor_Rotate_Left, 19.2f, 1);
    Motor_Init(&Motor_Rotate_Right, 19.2f, 1);

    // Guide wheel
    Motor_Init(&Motor_LGW, 36, 1);
    Motor_Init(&Motor_RGW, 36, 1);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);
}

// 状态机使用函数
void Take_TV_0(void) {
    TV_Out = 0;
}

void Take_TV_1(void) {
    TV_Out = 1;
}

void Take_TV_2(void) {
    TV_Out = 2;
}

void Take_Horizontal_Right(void) {
    TH_Move            = 1;
    Detected_Direction = 1;
}

void Take_Horizontal_Left(void) {
    TH_Move            = 2;
    Detected_Direction = 2;
}

void Take_Chassis_Detect_Right(void) {
    Chassis_Detect     = 1;
    Detected_Direction = 1;
}

void Take_Chassis_Detect_Left(void) {
    Chassis_Detect     = 2;
    Detected_Direction = 2;
}

void Take_Start_Get(void) {
    TH_Move = 0;
    TR_Get  = 2;
}

void Take_TV_Progress(void) {
    TV_Out = 3;
}

void Take_ON(void) {
    TAKE_ON;
}

void Take_Up(void) {
    TU_Up = 2;
}

void Take_Down(void) {
    TU_Up = 1;
}

void Take_Rotate_OFF(void) {
    TR_Get = 0;
}

void Take_OFF(void) {
    TAKE_OFF;
}

void Take_Catapult_On(void) {
    CATAPULT_ON;
}

void Take_Reset(void) {
    Chassis_Detect = 0;
    TAKE_OFF;
    CATAPULT_OFF;
    TR_Get         = 0;
    TV_Out         = 0;
    TU_Up          = 1;
    Chassis_State  = CHASSIS_NORMAL;
    Detected_State = 0;
}
