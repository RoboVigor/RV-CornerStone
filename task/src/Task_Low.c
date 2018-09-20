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
    TickType_t LastWakeTime = xTaskGetTickCount();  // 时钟
    int        Buffer[4];                           // 麦轮解算后四个麦轮各自的速度
    int        WheelSpeedRes[4];                    // 限幅后四个麦轮各自的速度
    float      kFeedback = 3.14 / 60;               // 转子的转速()换算成角速度(rad/s)，

    PID_Init(&CM1PID, 12, 0.2, 0, 4000); // 初始化麦轮角速度 PID
    PID_Init(&CM2PID, 12, 0.2, 0, 4000);
    PID_Init(&CM3PID, 12, 0.2, 0, 4000);
    PID_Init(&CM4PID, 12, 0.2, 0, 4000);
    PID_Init(&YawSpeedPID, 15, 0, 0, 1000); // 初始化 yaw 角速度 PID

    while (1) {
        yawSpeedFeed = mpu6500_data.gz / 16.4; // yaw 角速度反馈
        PID_Calculate(&YawSpeedPID, DBusData.ch1, yawSpeedFeed); // 计算 yaw 角速度 PID

        Chassis_Set_Wheel_Speed(DBusData.ch4, -DBusData.ch3, YawSpeedPID.output); //配置小车整体 XYZ 三个轴的速度
        Chassis_Update_Mecanum_Data(Buffer);                                      //麦轮的解算
        Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, MAXWHEELSPEED);          //限幅

        PID_Calculate(&CM1PID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback); // 计算麦轮角速度 PID
        PID_Calculate(&CM2PID, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback); // 因为电机和麦克纳姆轮解算对应错误，WheelSpeedRes[3] 和 WheelSpeedRes[1] 对换位置
        PID_Calculate(&CM3PID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        PID_Calculate(&CM4PID, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);

        Can_Set_CM_Current(CAN1, CM1PID.output, CM2PID.output, CM3PID.output, CM4PID.output); // 输出电流值到电调

        vTaskDelayUntil(&LastWakeTime, 100);
<<<<<<< HEAD

        continue;

        // yawSpeedFeed = mpu6500_data.gz / 16.4;
        // yawAngleFeed = EulerAngle.Yaw;
        // PID_Init(&YawAnglePID, 15, 0, 0, 660); // 15
        // PID_Init(&YawSpeedPID, 2, 0, 0, 660);  // 2

        PID_Calculate(&YawAnglePID, remoteData.rx, yawAngleFeed);
        PID_Calculate(&YawSpeedPID, YawAnglePID.output, yawSpeedFeed);
        Chassis_Set_Wheel_Speed(remoteData.ly, remoteData.lx, YawSpeedPID.output); //设定XYZ三个轴的速度

        // Chassis_Update_Mecanum_Data(Buffer); //麦轮的解算

        Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, CHASSIS_MAX_WHEEL_SPEED); //限幅

        // PID_Calculate(&CM1PID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback);
        // PID_Calculate(&CM2PID, WheelSpeedRes[1], Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&CM3PID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&CM4PID, WheelSpeedRes[3], Motor_Feedback.motor204Speed * kFeedback);

        // Can_Set_CM_Current(CAN1, CM1PID.output, -CM2PID.output, -CM3PID.output, CM4PID.output);

        // vTaskDelayUntil(&LastWakeTime, 50);
=======
>>>>>>> pid update(非最终版)
    }

    vTaskDelete(NULL);
}
