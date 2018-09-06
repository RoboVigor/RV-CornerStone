#include "main.h"

/**
 * @brief  LED闪烁任务 确认存活
 * @param  void *Parameters
 * @return void
 */

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}

/**
 * @brief  底盘@甩锅小车
 * @param  void *Parameters
 * @return void
 */

void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    int        ArmatureRotateSpeed[4], Buffer[4];
    int        kFeedback = 2 * 3.14 / 60;
    int        targetSpeed;

    while (1) {
        PANSpeedPIDInit(&CM1PID, 12, 0, 0);
        PANSpeedPIDInit(&CM2PID, 12, 0, 0);
        PANSpeedPIDInit(&CM3PID, 12, 0, 0);
        PANSpeedPIDInit(&CM4PID, 12, 0, 0);

        if (DBusData.ch4 > 20) {
            targetSpeed = 80;
        } else if (DBusData.ch4 < -20) {
            targetSpeed = -80;
        } else {
            targetSpeed = 0;
        }

        PID_Set_Pan_Speed(&CM1PID, targetSpeed, Motor_Feedback.motor201Speed * kFeedback);
        PID_Set_Pan_Speed(&CM2PID, targetSpeed, Motor_Feedback.motor202Speed * kFeedback);
        PID_Set_Pan_Speed(&CM3PID, targetSpeed, Motor_Feedback.motor203Speed * kFeedback);
        PID_Set_Pan_Speed(&CM4PID, targetSpeed, Motor_Feedback.motor204Speed * kFeedback);

        Can_Set_CM_Current(CAN1, CM1PID.PIDout, -CM2PID.PIDout, -CM3PID.PIDout, CM4PID.PIDout);

        vTaskDelayUntil(&LastWakeTime, 50);
    }

    vTaskDelete(NULL);
}
