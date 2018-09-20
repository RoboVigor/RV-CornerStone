#include "main.h"

// for debug pid
int      feedback1 = 0;
int      feedback2 = 0;
int      feedback3 = 0;
int      feedback4 = 0;
int      I_out     = 0;
int      P_out     = 0;
int      target    = 0;
uint16_t wuwuwu    = 0;

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

    // For debug pid
    float pTestWheel      = 5;
    float iTestWheel      = 0.2;
    float maxOutPut       = 4000;
    float pTestYaw        = 2;
    float iTestYaw        = 0;
    float currentLimitYaw = 660;
    //    float magicc            = 400;

    PID_Init(&CM1PID, pTestWheel, iTestWheel, 0, currentLimitWheel);
    PID_Init(&CM2PID, pTestWheel, iTestWheel, 0, currentLimitWheel);
    PID_Init(&CM3PID, pTestWheel, iTestWheel, 0, currentLimitWheel);
    PID_Init(&CM4PID, pTestWheel, iTestWheel, 0, currentLimitWheel);
    PID_Init(&YawAnglePID, 15, 0, 0, 660); // Angle
    // PID_Init(&YawSpeedPID, pTestYaw, iTestYaw, 0, currentLimitYaw); // Speed

    while (1) {
        /*For debug pid*/
        yawAngleFeed = EulerAngle.Yaw;                           // yaw 角度反馈
        yawSpeedFeed = mpu6500_data.gz / 16.4;                   // yaw 速度反馈
        PID_Calculate(&YawAnglePID, DBusData.ch1, yawAngleFeed); // Angle
        // PID_Calculate(&YawSpeedPID, DBusData.ch1, yawSpeedFeed); // Speed
        PID_Calculate(&YawSpeedPID, YawAnglePID.output, yawSpeedFeed);

        // Chassis_Set_Wheel_Speed(DBusData.ch4, DBusData.ch3, YawSpeedPID.output); // Speed
        Chassis_Set_Wheel_Speed(DBusData.ch4, -DBusData.ch3, YawSpeedPID.output); //设定XYZ三个轴的速度  // Speed
        Chassis_Update_Mecanum_Data(Buffer);                                      //麦轮的解算
        Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, MAXWHEELSPEED);          //限幅

        PID_Calculate(&CM1PID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback);
        PID_Calculate(&CM2PID, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback); // 因为电机和麦克纳姆轮解算对应错误，3 和 1 对换位置
        PID_Calculate(&CM3PID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        PID_Calculate(&CM4PID, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);

        // PID_Calculate(&CM1PID, magicc, Motor_Feedback.motor201Speed * kFeedback);
        // PID_Calculate(&CM2PID, magicc, Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&CM3PID, magicc, Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&CM4PID, magicc, Motor_Feedback.motor204Speed * kFeedback);

        // if (magic.value > -1000 && magic.value < 1000) { // 防止 CAN 线挂掉
        // Can_Set_CM_Current(CAN1, CM1PID.output, 0, 0, 0);
        Can_Set_CM_Current(CAN1, CM1PID.output, CM2PID.output, CM3PID.output, CM4PID.output);
        // }

        // feedback for jscope
        // feedback1 = Motor_Feedback.motor201Speed;
        // feedback2 = Motor_Feedback.motor202Speed;
        // feedback3 = -Motor_Feedback.motor203Speed;
        // feedback4 = Motor_Feedback.motor204Speed;
        // wuwuwu    = LastWakeTime;
        // target    = (int) WheelSpeedRes[0];
        // I_out     = (int) CM1PID.output_I;
        // P_out     = (int) CM1PID.output_P;

        vTaskDelayUntil(&LastWakeTime, 100);

        continue;

        // yawSpeedFeed = mpu6500_data.gz / 16.4;
        // yawAngleFeed = EulerAngle.Yaw;
        // PID_Init(&YawAnglePID, 15, 0, 0, 660); // 15
        // PID_Init(&YawSpeedPID, 2, 0, 0, 660);  // 2

        // PID_Calculate(&YawAnglePID, DBusData.ch1, yawAngleFeed);
        // PID_Calculate(&YawSpeedPID, YawAnglePID.output, yawSpeedFeed);
        // Chassis_Set_Wheel_Speed(DBusData.ch4, DBusData.ch3, YawSpeedPID.output); //设定XYZ三个轴的速度

        // Chassis_Update_Mecanum_Data(Buffer); //麦轮的解算

        // Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, MAXWHEELSPEED); //限幅

        // PID_Calculate(&CM1PID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback);
        // PID_Calculate(&CM2PID, WheelSpeedRes[1], Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&CM3PID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&CM4PID, WheelSpeedRes[3], Motor_Feedback.motor204Speed * kFeedback);

        // Can_Set_CM_Current(CAN1, CM1PID.output, -CM2PID.output, -CM3PID.output, CM4PID.output);

        // vTaskDelayUntil(&LastWakeTime, 50);
    }

    vTaskDelete(NULL);
}
