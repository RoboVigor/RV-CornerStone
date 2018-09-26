#include "main.h"

// For jlink debug
int debugA;
int debugB;
int debugC;
int debugD;
int debugE;
int debugF;
int debugG;
int debugH;

int a, b, c, d, e, f, g, h;

int yawAngleMode = 2;

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

    // mpu6500_data.gx_offset  = (short) 0.406409323;
    // mpu6500_data.gy_offset  = (short) -2.91589163;
    // mpu6500_data.gz_offset  = (short) 15.75639464;
    // EulerAngle.Yaw_offset   = 0;
    // EulerAngle.Pitch_offset = 0;

    // 初始化麦轮角速度 PID
    PID_Init(&PID_LFCM, 12, 0.2, 0, 4000, 1500); // 12 0.2
    PID_Init(&PID_LBCM, 12, 0.2, 0, 4000, 1500);
    PID_Init(&PID_RBCM, 12, 0.2, 0, 4000, 1500);
    PID_Init(&PID_RFCM, 12, 0.2, 0, 4000, 1500);

    PID_Init(&YawAnglePID, magic.value, 0, 0, 1000, 1000); // 初始化 yaw 角度 PID(2)
    PID_Init(&YawSpeedPID1, 8, 0, 0, 4000, 1000);          // 初始化 yaw 角速度 PID(15)
    PID_Init(&YawSpeedPID2, 8, 0, 0, 4000, 1000);          // 初始化 yaw 角速度 PID(15)
    PID_Init(&YawSpeedPID, 8, 0, 0, 4000, 1000);           // 初始化 yaw 角速度 PID(15)

    while (1) {
        // 陀螺仪 - 可能需要单独维护
        // For magic debug
        // PID_LFCM.maxOutput_I = magic.value;
        // PID_LBCM.maxOutput_I = magic.value;
        // PID_RFCM.maxOutput_I = magic.value;
        // PID_RBCM.maxOutput_I = magic.value;

        yawAngleFeed = EulerAngle.Yaw;         // yaw 角度反馈
        yawSpeedFeed = mpu6500_data.gz / 16.4; // yaw 角速度反馈

        // YAW ANGLE HACK MODE
        if (ABS(DBusData.ch1) < 5) {
            // xEventGroupSetBits(EventGroupHandler_YawAngleMode, EVENTBIT1);
            if (yawAngleMode == 2) {
                YawAnglePID.output_I  = 0;
                YawSpeedPID1.output_I = 0;
                yawAngleTarget        = yawAngleFeed;
                yawAngleMode          = 1;
            }
            PID_Calculate(&YawAnglePID, yawAngleTarget, yawAngleFeed);                // 计算 yaw 角度 PID
            PID_Calculate(&YawSpeedPID, YawAnglePID.output, yawSpeedFeed);            // 计算 yaw 角速度 PID
            Chassis_Set_Wheel_Speed(DBusData.ch4, -DBusData.ch3, YawSpeedPID.output); // 配置小车整体 XYZ 三个轴的速度
        } else {
            // xEventGroupSetBits(EventGroupHandler_YawAngleMode, EVENTBIT2);
            if (yawAngleMode == 1) {
                YawSpeedPID2.output_I = 0;
                yawAngleMode          = 2;
            }
            PID_Calculate(&YawSpeedPID, DBusData.ch1, yawSpeedFeed);                  // 计算 yaw 角速度 PID
            Chassis_Set_Wheel_Speed(DBusData.ch4, -DBusData.ch3, YawSpeedPID.output); // 配置小车整体 XYZ 三个轴的速度
        }

        Chassis_Update_Mecanum_Data(Buffer);                             // 麦轮的解算
        Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, MAXWHEELSPEED); // 限幅

        // 计算麦轮角速度 PID
        // 因为电机和麦克纳姆轮解算对应错误，WheelSpeedRes[3] 和 WheelSpeedRes[1] 对换位置
<<<<<<< HEAD
        // PID_Calculate(&LFCMPID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback); // 位置 PID
        // PID_Calculate(&LBCMPID, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&RBCMPID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&RFCMPID, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);
        Increment_PID_Calculate(&LFCMPID, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback); // 增量 PID
        Increment_PID_Calculate(&LBCMPID, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback);
        Increment_PID_Calculate(&RBCMPID, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        Increment_PID_Calculate(&RFCMPID, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);

        Can_Set_CM_Current(CAN1, LFCMPID.output, LBCMPID.output, RBCMPID.output, RFCMPID.output); // 输出电流值到电调

<<<<<<< HEAD
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
=======
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
>>>>>>> 0.2.20 使用了陀螺仪（陀螺仪代码待维护）
=======
        // PID_Calculate(&PID_LFCM, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback); // 位置 PID
        // PID_Calculate(&PID_LBCM, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback);
        // PID_Calculate(&PID_RBCM, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        // PID_Calculate(&PID_RFCM, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);
        Increment_PID_Calculate(&PID_LFCM, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback); // 增量 PID
        Increment_PID_Calculate(&PID_LBCM, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback);
        Increment_PID_Calculate(&PID_RBCM, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        Increment_PID_Calculate(&PID_RFCM, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);

        Can_Set_CM_Current(CAN1, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output); // 输出电流值到电调

        // For jlink debug
        debugA = (int) YawAnglePID.output;
        debugB = (int) YawSpeedPID1.output;
        debugC = (int) yawAngleMode;
        debugD = (int) yawAngleTarget;

        debugE = (int) EulerAngle.Roll;
        debugF = (int) DBusData.ch1;
        debugG = (int) EulerAngle.Yaw;
        // debugH = (int) CM4PID.output;
        vTaskDelayUntil(&LastWakeTime, 100);
>>>>>>> angle pid update
    }

    vTaskDelete(NULL);
}

void Task_Event_Group(void *Parameters) {
    EventBits_t lastMode    = 0;
    EventBits_t currentMode = 0;

    while (1) {
        currentMode = xEventGroupWaitBits(EventGroupHandler_YawAngleMode, EVENTBITALL, pdTRUE, pdTRUE, portMAX_DELAY);
        if (currentMode == EVENTBIT1) {
            YawAnglePID.output_I  = 0;
            YawSpeedPID1.output_I = 0;
            YawSpeedPID2.output_I = 0;
            yawAngleTarget        = yawAngleFeed;
            xEventGroupSetBits(EventGroupHandler_YawAngleMode, EVENTBIT3);
        } else if (currentMode == EVENTBIT2) {
            YawAnglePID.output_I  = 0;
            YawSpeedPID1.output_I = 0;
            YawSpeedPID2.output_I = 0;
            xEventGroupSetBits(EventGroupHandler_YawAngleMode, EVENTBIT3);
        }
        debugH = (int) currentMode;
    }
}

void Task_Mpu6500(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟

    while (1) {
        Gyroscope_Update_Angle_Data(); // 陀螺仪姿态解算
        vTaskDelayUntil(&LastWakeTime, 100);
    }
    vTaskDelete(NULL);
}