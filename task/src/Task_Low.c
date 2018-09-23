#include "main.h"

// For Jlink debug
int a, b, c, d, e, f, g, h;

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
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    int        Buffer[4];                          // 麦轮解算后四个麦轮各自的速度
    int        WheelSpeedRes[4];                   // 限幅后四个麦轮各自的速度
    float      kFeedback = 3.14 / 60;              // 转子的转速(round/min)换算成角速度(rad/s)

    // 陀螺仪相关参数

    // float yawAngleOffset              = 0;
    // float yawAngleFeedLast            = 0;
    // float yawAngleDiff                = 0;
    // float yawAngleThreshold           = 0.03;
    // float yawAngleOffsetSampleCounter = 0;
    // float yawAngleOffsetSample        = 0;

    int yawAngleMode = 0;

    // mpu6500_data.gx_offset  = (short) 0.406409323;
    // mpu6500_data.gy_offset  = (short) -2.91589163;
    // mpu6500_data.gz_offset  = (short) 15.75639464;
    // EulerAngle.Yaw_offset   = 0;
    // EulerAngle.Pitch_offset = 0;

    // 初始化麦轮角速度 PID
    PID_Init(&LFCMPID, 12, 0.2, 0, 4000, 1500); // 12 0.2
    PID_Init(&LBCMPID, 12, 0.2, 0, 4000, 1500);
    PID_Init(&RBCMPID, 12, 0.2, 0, 4000, 1500);
    PID_Init(&RFCMPID, 12, 0.2, 0, 4000, 1500);

    PID_Init(&YawAnglePID, 2, 0, 0, 1000, 1000);  // 初始化 yaw 角度 PID(2)
    PID_Init(&YawSpeedPID, 10, 0, 0, 1000, 1000); // 初始化 yaw 角速度 PID(15)

    while (1) {
        // 陀螺仪 - 可能需要单独维护
        Gyroscope_Update_Angle_Data(); // 陀螺仪姿态解算

        // For magic debug
        // LFCMPID.maxOutput_I = magic.value;
        // LBCMPID.maxOutput_I = magic.value;
        // RFCMPID.maxOutput_I = magic.value;
        // RBCMPID.maxOutput_I = magic.value;

        yawAngleFeed = EulerAngle.Yaw;         // yaw 角度反馈
        yawSpeedFeed = mpu6500_data.gz / 16.4; // yaw 角速度反馈

        // yawAngleDiff = yawAngleFeed - yawAngleFeedLast;
        // if (ABS(yawAngleDiff) <= yawAngleThreshold) {
        //     yawAngleFeedLast = yawAngleFeed;
        //     if (yawAngleOffsetSampleCounter < 100) {
        //         yawAngleOffsetSample += yawAngleDiff;
        //         yawAngleOffsetSampleCounter++;
        //         continue;
        //     } else if (yawAngleOffsetSampleCounter == 100) {
        //         yawAngleOffset += yawAngleOffsetSample / yawAngleOffsetSampleCounter;
        //         yawAngleOffsetSampleCounter++;
        //         DBusData.ch1 = 0;
        //         DBusData.ch3 = 0;
        //         DBusData.ch4 = 0;
        //     }
        //     yawAngleOffset += yawAngleDiff;
        //     yawAngleFeed = yawAngleFeedLast;
        // } else {
        //     yawAngleOffset += yawAngleDiff;
        //     yawAngleFeedLast = yawAngleFeed;
        // }
        // if (ABS(yawAngleDiff) <= yawAngleThreshold) {
        //     Increment_PID_Calculate(&YawAnglePID, yawAngleFeedLast, yawAngleFeed);   // 计算 yaw 角度 PID
        //     Increment_PID_Calculate(&YawSpeedPID, YawAnglePID.output, yawSpeedFeed); // 计算 yaw 角速度 PID
        // } else {
        //     Increment_PID_Calculate(&YawSpeedPID, DBusData.ch1, yawSpeedFeed); // 计算 yaw 角速度 PID
        // }

        // YAW ANGLE HACK MODE
        if (ABS(DBusData.ch1) < 5) {
            if (yawAngleMode == 2) {
                yawAngleTarget = yawAngleFeed;
                yawAngleMode   = 1;
            }
            Increment_PID_Calculate(&YawAnglePID, yawAngleTarget, yawAngleFeed);     // 计算 yaw 角度 PID
            Increment_PID_Calculate(&YawSpeedPID, YawAnglePID.output, yawSpeedFeed); // 计算 yaw 角速度 PID
        } else {
            if (yawAngleMode == 1) {
                YawAnglePID.output_I = 0;
                YawSpeedPID.output_I = 0;
                yawAngleTarget       = 2;
            }
            Increment_PID_Calculate(&YawSpeedPID, DBusData.ch1, yawSpeedFeed); // 计算 yaw 角速度 PID
        }

        Chassis_Set_Wheel_Speed(DBusData.ch4, -DBusData.ch3, YawSpeedPID.output); // 配置小车整体 XYZ 三个轴的速度
        Chassis_Update_Mecanum_Data(Buffer);                                      // 麦轮的解算
        Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, MAXWHEELSPEED);          // 限幅

        // 计算麦轮角速度 PID
        // 因为电机和麦克纳姆轮解算对应错误，WheelSpeedRes[3] 和 WheelSpeedRes[1] 对换位置
        // PID_Calculate(&LFCMPID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback); // 位置 PID
        // PID_Calculate(&LBCMPID, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&RBCMPID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&RFCMPID, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);
        Increment_PID_Calculate(&LFCMPID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback); // 增量 PID
        Increment_PID_Calculate(&LBCMPID, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback);
        Increment_PID_Calculate(&RBCMPID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        Increment_PID_Calculate(&RFCMPID, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);

        Can_Set_CM_Current(CAN1, LFCMPID.output, LBCMPID.output, RBCMPID.output, RFCMPID.output); // 输出电流值到电调

        // For Jlink debug
        a = (int) EulerAngle.Yaw;
        // b = (int) LBCMPID.output;
        // c = (int) RBCMPID.output;
        // d = (int) RFCMPID.output;

        // e = (int) CM1PID.output;
        // f = (int) CM2PID.output;
        // g = (int) CM3PID.output;
        // h = (int) CM4PID.output;

        vTaskDelayUntil(&LastWakeTime, 20);
    }

    vTaskDelete(NULL);
}
