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

void Take_Horizontal(void) {
    TH_Move = 1;
    if(LSR_State == 1) {
        // Chassis_State = CHASSIS_DETECT_RIGHT;
        GPIO_SetBits(GPIOD, GPIO_Pin_15);
    }
}

void Take_Start_Get(void) {
    TH_Move = 0;
    // Chassis_State = CHASSIS_NORMAL;
    GPIO_ResetBits(GPIOD, GPIO_Pin_15);
    vTaskDelay(500);
    ROTATE_ON;
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
    vTaskDelay(300);
    CATAPULT_OFF;
    CATAPULT_ON;
    vTaskDelay(300);
    CATAPULT_OFF;
}

void Take_Reset(void) {
    TAKE_OFF;
    CATAPULT_OFF;
    ROTATE_OFF;
    TH_Move = 0;
    TV_Out = 0;
    TU_Up = 1;
    Chassis_State = CHASSIS_NORMAL;
}

