/**
 * @brief 功能任务
 */

#include "main.h"

/**
 * @brief  LED闪烁任务 确认存活
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
 */
int  yawAngleMode = 2;
void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    int        Buffer[4];                          // 麦轮解算后四个麦轮各自的速度
    int        WheelSpeedRes[4];                   // 限幅后四个麦轮各自的速度
    float      kFeedback = 3.14 / 60;              // 转子的转速(round/min)换算成角速度(rad/s)

    // 初始化麦轮角速度 PID
    PID_Init(&PID_LFCM, 12, 0.2, 0, 4000, 1500); // 12 0.2
    PID_Init(&PID_LBCM, 12, 0.2, 0, 4000, 1500);
    PID_Init(&PID_RBCM, 12, 0.2, 0, 4000, 1500);
    PID_Init(&PID_RFCM, 12, 0.2, 0, 4000, 1500);

    PID_Init(&PID_YawAngle, 8, 0, 0, 1000, 1000); // 初始化 yaw 角度 PID(2)
    PID_Init(&PID_YawSpeed, 8, 0, 0, 4000, 1000); // 初始化 yaw 角速度 PID(15)

    while (1) {

        // 设置反馈值
        yawAngleFeed = EulerAngle.Yaw;         // yaw 角度反馈
        yawSpeedFeed = mpu6500_data.gz / 16.4; // yaw 角速度反馈

        // 运动模式判断 1直线 2转弯
        if (ABS(remoteData.ch1) < 5) {
            //直线模式
            // xEventGroupSetBits(EventGroupHandler_YawAngleMode, EVENTBIT1);
            if (yawAngleMode == 2) {
                PID_YawAngle.output_I = 0;
                PID_YawSpeed.output_I = 0;
                yawAngleTarget        = yawAngleFeed;
            }
            yawAngleMode = 1;
            PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngleFeed);                    // 计算 yaw 角度 PID
            PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeedFeed);               // 计算 yaw 角速度 PID
            Chassis_Set_Wheel_Speed(remoteData.ch4, -remoteData.ch3, PID_YawSpeed.output); // 配置小车整体 XYZ 三个轴的速度
        } else {
            //转弯模式
            // xEventGroupSetBits(EventGroupHandler_YawAngleMode, EVENTBIT2);
            if (yawAngleMode == 1) {
                PID_YawAngle.output_I = 0;
                PID_YawSpeed.output_I = 0;
            }
            yawAngleMode = 2;
            PID_Calculate(&PID_YawSpeed, remoteData.ch1, yawSpeedFeed);                    // 计算 yaw 角速度 PID
            Chassis_Set_Wheel_Speed(remoteData.ch4, -remoteData.ch3, PID_YawSpeed.output); // 配置小车整体 XYZ 三个轴的速度
        }

        Chassis_Update_Mecanum_Data(Buffer);                                       // 麦轮解算
        Chassis_Limit_Wheel_Speed(Buffer, WheelSpeedRes, CHASSIS_MAX_WHEEL_SPEED); // 限幅

        PID_Calculate(&PID_LFCM, WheelSpeedRes[0], Motor_Feedback.motor201Speed * kFeedback);
        PID_Calculate(&PID_LBCM, WheelSpeedRes[3], Motor_Feedback.motor202Speed * kFeedback);
        PID_Calculate(&PID_RBCM, WheelSpeedRes[2], Motor_Feedback.motor203Speed * kFeedback);
        PID_Calculate(&PID_RFCM, WheelSpeedRes[1], Motor_Feedback.motor204Speed * kFeedback);

        Can_Set_CM_Current(CAN1, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output); // 输出电流值到电调

        vTaskDelayUntil(&LastWakeTime, 100);
    }

    vTaskDelete(NULL);
}

/**
 * @brief 垃圾任务
 */
// void Task_Event_Group(void *Parameters) {
//     EventBits_t lastMode    = 0;
//     EventBits_t currentMode = 0;

//     while (1) {
//         currentMode = xEventGroupWaitBits(EventGroupHandler_YawAngleMode, EVENTBITALL, pdTRUE, pdTRUE, portMAX_DELAY);
//         if (currentMode == EVENTBIT1) {
//             PID_YawAngle.output_I  = 0;
//             PID_YawSpeed1.output_I = 0;
//             PID_YawSpeed2.output_I = 0;
//             yawAngleTarget         = yawAngleFeed;
//             xEventGroupSetBits(EventGroupHandler_YawAngleMode, EVENTBIT3);
//         } else if (currentMode == EVENTBIT2) {
//             PID_YawAngle.output_I  = 0;
//             PID_YawSpeed1.output_I = 0;
//             PID_YawSpeed2.output_I = 0;
//             xEventGroupSetBits(EventGroupHandler_YawAngleMode, EVENTBIT3);
//         }
//     }
// }

/**
 * @brief  安全模式
 */
void Task_Safe_Mode(void *Parameters) {

    while (1) {
        if (remoteData.switchRight == 2) {
            Can_Set_CM_Current(CAN1, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }

    vTaskDelete(NULL);
}
