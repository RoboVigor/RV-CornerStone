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
    Motor_Init(&Motor_TH, 36, 1); // 2006
    Motor_Init(&Motor_TV, 19.2f, 1); // 3510

    // 抬升
    Motor_Init(&Motor_Upthrow1, 19.2f, 1);
    Motor_Init(&Motor_Upthrow2, 19.2f, 1);

    // Guide wheel
    Motor_Init(&Motor_LGW, 36, 1);
    Motor_Init(&Motor_RGW, 36, 1);

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // 初始化串口调试数据
    Magic_Init(&magic, 0);
}


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
    TH_Move = 1;
}

void Take_Horizontal_Left(void) {
    TH_Move = 2;
}

void Take_Chassis_Detect(void) {
    Chassis_Detect = 1;
}

void Take_Start_Get(void) {
    TH_Move = 0;
    Chassis_Detect = 0;
    vTaskDelay(500);
    ROTATE_ON;
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
    ROTATE_OFF;
}

void Take_OFF(void) {
    TAKE_OFF;
}

void Take_Catapult(void) {
    CATAPULT_ON;
    vTaskDelay(500);
    CATAPULT_OFF;
    vTaskDelay(500);
    CATAPULT_ON;
    vTaskDelay(500);
    CATAPULT_OFF;
}

void Take_Reset(void) {
    TAKE_OFF;
    vTaskDelay(1000);
    CATAPULT_OFF;
    vTaskDelay(1000);
    ROTATE_OFF;
    vTaskDelay(1000);
    TV_Out = 0;
    vTaskDelay(4000);
    TH_Move = 2;
    vTaskDelay(4000);
    TU_Up = 1;
    vTaskDelay(4000);
    Chassis_Detect = 0;
    vTaskDelay(4000);
    Chassis_State = CHASSIS_NORMAL;
}

