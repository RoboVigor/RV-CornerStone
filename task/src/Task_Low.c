#include "main.h"

/**
 * @brief  LED闪烁任务 确认存活
 * @param  void *Parameters
 * @return void
 */
// int diff    = 0;
int _target = 0;

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
    float kFeedback        = 3.14 / 60;
    int   minCurrentToMove = 600;

    // int timesToMove = 0;

    PID_Init(&ChassisAnglePID1, 1.5, 0, 0, 100); // 1.5// 0.5  -1.755   0.7   1.36
    PID_Init(&CM1PID, 60, 0, 0, 800);            // 400//0.04//28
    // PID_Init(&CM2PID, 0, 0, 0, 1000);
    // PID_Init(&CM3PID, 0, 0, 0, 1000);
    // PID_Init(&CM4PID, 0, 0, 0, 1000);
    CM1_Encoder.ecdBias = Motor_Feedback.motor201Angle;
    CAN_Get_Encoder_Bias(&CM1_Encoder);

    while (1) {

        // PID_Calculate(&CM1PID, magic.value, Motor_Feedback.motor201Speed * kFeedback);
        // PID_Calculate(&CM2PID, magic.value, Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&CM3PID, magic.value, Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&CM4PID, magic.value, Motor_Feedback.motor204Speed * kFeedback);

        // if (magic.value > 0 && magic.value < 2000) { // 防止 CAN 线挂断
        //     Can_Set_CM_Current(CAN1, magic.value, 0, 0, 0);
        // }
        // vTaskDelayUntil(&LastWakeTime, 100);

        // continue;

        CAN_Update_Encoder_Data(&CM1_Encoder, Motor_Feedback.motor201Angle);

        // debug1 = CM1_Encoder.ecdAngle;
        // debug2 = CM1_Encoder.rawValue;
        // debug3 = CM1_Encoder.roundCnt;
        // CM1PID.p = magic.value;

        // else if (Motor_Feedback.motor201Speed > 2) {
        //     if (CM1PID.output >= -minCurrentToMove && CM1PID.output < 0) CM1PID.output = -minCurrentToMove;
        //     if (CM1PID.output <= minCurrentToMove && CM1PID.output > 0) CM1PID.output = minCurrentToMove;
        // }

        PID_Calculate(&ChassisAnglePID1, -720, CM1_Encoder.ecdAngle / 19);
        PID_Calculate(&CM1PID, ChassisAnglePID1.output, Motor_Feedback.motor201Speed * kFeedback);

        //逐渐增大电流
        // if (CM1PID.output >= -minCurrentToMove && CM1PID.output < 0) {
        //     CM1PID.output += timesToMove * 100;
        //     timesToMove++;
        // } else if (CM1PID.output <= minCurrentToMove && CM1PID.output > 0) {
        //     CM1PID.output += timesToMove * 100;
        //     timesToMove++;
        // }
        // if (CM1PID.output >= -minCurrentToMove && CM1PID.output < 0) CM1PID.output = -minCurrentToMove;
        // if (CM1PID.output <= minCurrentToMove && CM1PID.output > 0) CM1PID.output = minCurrentToMove;
        // Can_Set_CM_Current(CAN1, 650, 0, 0, 0);
        _Set_CM_Current(CM1PID.output); // 225
        vTaskDelayUntil(&LastWakeTime, 10);

        // yawSpeedFeed = mpu6500_data.gz / 16.4;
        // yawAngleFeed = EulerAngle.Yaw;
        // PID_Init(&YawAnglePID, 15, 0, 0, 660);
        // PID_Init(&YawSpeedPID, 2, 0, 0, 660);

        // PID_Calculate(&YawAnglePID, DBusData.ch1, yawAngleFeed);
        // PID_Calculate(&YawSpeedPID, YawAnglePID.output, yawSpeedFeed);
        // Chassis_Set_Wheel_Speed(DBusData.ch4, DBusData.ch3, YawSpeedPID.output); //设定XYZ三个轴的速度

        // Chassis_Update_Mecanum_Data(Buffer); //麦轮的解算

        // Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, MAXWHEELSPEED); //限幅

        // PID_Calculate(&CM1PID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback);
        // PID_Calculate(&CM2PID, WheelSpeedRes[1], Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&CM3PID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&CM4PID, WheelSpeedRes[3], Motor_Feedback.motor204Speed * kFeedback);
        // diff = Motor_Feedback.motor201Speed * kFeedback;
        // Can_Set_CM_Current(CAN1, CM1PID.output, -CM2PID.output, -CM3PID.output, CM4PID.output);
    }

    vTaskDelete(NULL);
}

void _Set_CM_Current(int16_t i_201) {
    int minCurrent    = 700;  // 250
    int currentToMove = 2000; // 1500
    int threshold     = 600;
    if (ABS(i_201) > threshold) {
        if (Motor_Feedback.motor201Speed == 0) {
            Can_Set_CM_Current(CAN1, currentToMove, 0, 0, 0);
        } else {
            if (i_201 < 0 && i_201 > -minCurrent) i_201 = -minCurrent;
            if (i_201 > 0 && i_201 < minCurrent) i_201 = minCurrent;

            Can_Set_CM_Current(CAN1, i_201, 0, 0, 0);
        }
    } else {
        Can_Set_CM_Current(CAN1, 0, 0, 0, 0);
    }
}
