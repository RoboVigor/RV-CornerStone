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
    int        WheelSpeedRes[4], Buffer[4];
    float      kFeedback = 3.14 / 60;

    PID_Init(&CM1PID, 3, 0, 0, 1000);
    PID_Init(&CM2PID, 0, 0, 0, 1000);
    PID_Init(&CM3PID, 0, 0, 0, 1000);
    PID_Init(&CM4PID, 0, 0, 0, 1000);

    while (1) {

        // PID_Calculate(&CM1PID, magic.value, Motor_Feedback.motor201Speed * kFeedback);
        // PID_Calculate(&CM2PID, magic.value, Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&CM3PID, magic.value, Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&CM4PID, magic.value, Motor_Feedback.motor204Speed * kFeedback);

        if (magic.value > 0 && magic.value < 2000) { // 防止 CAN 线挂断
            Can_Set_CM_Current(CAN1, magic.value, 0, 0, 0);
        }
        vTaskDelayUntil(&LastWakeTime, 100);

        continue;

        yawSpeedFeed = mpu6500_data.gz / 16.4;
        yawAngleFeed = EulerAngle.Yaw;
        PID_Init(&YawAnglePID, 15, 0, 0, 660);
        PID_Init(&YawSpeedPID, 2, 0, 0, 660);

        PID_Calculate(&YawAnglePID, DBusData.ch1, yawAngleFeed);
        PID_Calculate(&YawSpeedPID, YawAnglePID.output, yawSpeedFeed);
        Chassis_Set_Wheel_Speed(DBusData.ch4, DBusData.ch3, YawSpeedPID.output); //设定XYZ三个轴的速度

        Chassis_Update_Mecanum_Data(Buffer); //麦轮的解算

        Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, MAXWHEELSPEED); //限幅

        PID_Calculate(&CM1PID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback);
        PID_Calculate(&CM2PID, WheelSpeedRes[1], Motor_Feedback.motor202Speed * kFeedback);
        PID_Calculate(&CM3PID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        PID_Calculate(&CM4PID, WheelSpeedRes[3], Motor_Feedback.motor204Speed * kFeedback);

        Can_Set_CM_Current(CAN1, CM1PID.output, -CM2PID.output, -CM3PID.output, CM4PID.output);

        vTaskDelayUntil(&LastWakeTime, 50);
    }

    vTaskDelete(NULL);
}
