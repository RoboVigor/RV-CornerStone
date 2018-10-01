#include "main.h"

/**
 * @brief  LED闪烁任务 确认存活
 * @param  void *Parameters
 * @return void
 */
// int diff    = 0;
int _target = 0;
int debug4  = 0;
int debug5  = 0;
int debug6  = 0;
int debug7  = 0;
int debug8  = 0;

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}

/**
 * @brief  底盘@三星轮组
 * @param  void *Parameters
 * @return void
 */
void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    // int        WheelSpeedRes[4], Buffer[4];
    float kFeedback = 3.14 / 60;
    // int   minCurrentToMove = 600;
    // debug
    int target          = 0;
    int target12        = 0;
    int target34        = 0;
    int lastSwitchValue = 0;
    // int timesToMove = 0;

    PID_Init(&ChassisAnglePID1, 2, 0, 0, 100); // 1.5
    PID_Init(&ChassisAnglePID2, 2, 0, 0, 100); // 1.5
    PID_Init(&ChassisAnglePID3, 2, 0, 0, 100); // 1.5
    PID_Init(&ChassisAnglePID4, 2, 0, 0, 100); // 1.5

    PID_Init(&CM1PID, 55, 0, 0, 8000); // 35   0.01 //空载情况最佳状态p=35  下地状态p=55时大致可以驱动
    PID_Init(&CM2PID, 55, 0, 0, 8000); // 35   0.01
    PID_Init(&CM3PID, 55, 0, 0, 8000); // 35   0.01
    PID_Init(&CM4PID, 55, 0, 0, 8000); // 35   0.01

    // CM1PID.maxOutput_I = 5000;
    // PID_Init(&CM2PID, 0, 0, 0, 1000);
    // PID_Init(&CM3PID, 0, 0, 0, 1000);
    // PID_Init(&CM4PID, 0, 0, 0, 1000);
    CM1_Encoder.ecdBias = Motor_Feedback.motor201Angle;
    CM2_Encoder.ecdBias = Motor_Feedback.motor202Angle;
    CM3_Encoder.ecdBias = Motor_Feedback.motor203Angle;
    CM4_Encoder.ecdBias = Motor_Feedback.motor204Angle;

    CAN_Get_Encoder_Bias(&CM1_Encoder);
    CAN_Get_Encoder_Bias(&CM2_Encoder);
    CAN_Get_Encoder_Bias(&CM3_Encoder);
    CAN_Get_Encoder_Bias(&CM4_Encoder);
    // vTaskDelayUntil(&LastWakeTime, 1000);

    while (1) {
        // ChassisAnglePID1.p = magic.value / 10;
        // ChassisAnglePID2.p = magic.value / 10;
        // ChassisAnglePID3.p = magic.value / 10;
        // ChassisAnglePID4.p = magic.value / 10;

        if (DBusData.switchRight == 2) {
            Can_Set_CM_Current(CAN1, 0, 0, 0, 0);
            // Can2_Set_CM_Current(CAN2, 0, 0, 0, 0);
            continue;
        }
        if (DBusData.switchLeft == 3) {
            lastSwitchValue = 3;
        } else if (DBusData.switchLeft == 1 && lastSwitchValue == 3) {
            if (DBusData.switchRight == 1) {
                target34 = 530 + 162;
                target12 = 162;
            } else if (DBusData.switchRight == 3) {
                target12 = -530;
                target34 = ;
            }
            lastSwitchValue = 1;
        } else if (DBusData.switchLeft == 2 && lastSwitchValue == 3) {
            if (DBusData.switchRight == 1) {
                target34 = 162;
                target12 = 530;
            } else if (DBusData.switchRight == 3) {
                target12 = target;
                target34 = ;
            }
            lastSwitchValue = 2;
        }

        CAN_Update_Encoder_Data(&CM1_Encoder, Motor_Feedback.motor201Angle);
        CAN_Update_Encoder_Data(&CM2_Encoder, Motor_Feedback.motor202Angle);
        CAN_Update_Encoder_Data(&CM3_Encoder, Motor_Feedback.motor203Angle);
        CAN_Update_Encoder_Data(&CM4_Encoder, Motor_Feedback.motor204Angle);

        PID_Calculate(&ChassisAnglePID1, -target12, CM1_Encoder.ecdAngle / 19);
        PID_Calculate(&ChassisAnglePID2, target12, CM2_Encoder.ecdAngle / 19);
        PID_Calculate(&ChassisAnglePID3, target34, CM3_Encoder.ecdAngle / 19);
        PID_Calculate(&ChassisAnglePID4, -target34, CM4_Encoder.ecdAngle / 19);

        PID_Calculate(&CM1PID, ChassisAnglePID1.output, Motor_Feedback.motor201Speed * kFeedback);
        PID_Calculate(&CM2PID, ChassisAnglePID2.output, Motor_Feedback.motor202Speed * kFeedback);
        PID_Calculate(&CM3PID, ChassisAnglePID3.output, Motor_Feedback.motor203Speed * kFeedback);
        PID_Calculate(&CM4PID, ChassisAnglePID4.output, Motor_Feedback.motor204Speed * kFeedback);

        // debug4 = CM1_Encoder.ecdAngle / 19;
        // debug5 = ChassisAnglePID1.output;
        // debug6 = Motor_Feedback.motor201Speed * kFeedback;
        // debug7 = CM1PID.output;
        // debug8 = 720;

        // _Set_CM_Current(-200, 200, 200, -200);
        // _Set_CM_Current(-CM1PID.output, CM2PID.output, CM3PID.output, -CM4PID.output);
        Can_Set_CM_Current(CAN1, CM1PID.output, CM2PID.output, CM3PID.output, CM4PID.output);
        // _Set_CM_Current(0, 0, 0, 0);

        // Can_Set_CM_Current(CAN1, 0, 0, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 5);
    }

    vTaskDelete(NULL);
}
void Task_Wheel(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        Can2_Set_CM_Current(CAN2, 500, 0, 0, 0);
        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

// void _Set_CM_Current(int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204) {
//     i_201 = Regulate_Current(i_201, Motor_Feedback.motor201Speed);
//     i_202 = Regulate_Current(i_202, Motor_Feedback.motor202Speed);
//     i_203 = Regulate_Current(i_203, Motor_Feedback.motor203Speed);
//     i_204 = Regulate_Current(i_204, Motor_Feedback.motor204Speed);
//     Can_Set_CM_Current(CAN1, i_201, i_202, i_203, i_204);
// }

// int Regulate_Current(int current, int speed) {
//     int currentToMove = 1200;
//     int minCurrent    = magic.value;
//     if (current == 0) return 0;
//     if (speed == 0) {
//         current = Wang(current, currentToMove);
//     } else {
//         current = Wang(current, minCurrent);
//     }
//     return current;
// }

// int Wang(int val, int min) {
//     if (val < 0 && val > -min) val = -min;
//     if (val > 0 && val < min) val = min;
//     return val;
// }