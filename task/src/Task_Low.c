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
int  groupMode              = 0;
int  targetBackGroupOffset  = 0;
int  targetFrontGroupOffset = 0;
void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      kFeedback    = 3.14 / 60;

    // 轮组变量
    int target           = 0;
    int targetBackGroup  = 0;
    int targetFrontGroup = 0;
    int lastSwitchLeft   = 0;

    // 硬编码轮组运动参数
    int targetFrontGroupPreset[] = {0, 700, 600, 700, 0};
    int targetBackGroupPreset[]  = {0, 100, 600, 100, 0};

    // 角度PID
    PID_Init(&ChassisAnglePID1, 1.5, 0, 0, 100); // 1.5
    PID_Init(&ChassisAnglePID2, 1.5, 0, 0, 100); // 1.5
    PID_Init(&ChassisAnglePID3, 1.5, 0, 0, 100); // 1.5
    PID_Init(&ChassisAnglePID4, 1.5, 0, 0, 100); // 1.5

    // 速度PID 空载情况最佳状态p=35  下地状态p=55时大致可以驱动
    PID_Init(&CM1PID, 55, 0, 0, 8000); // 35   0.01
    PID_Init(&CM2PID, 55, 0, 0, 8000); // 35   0.01
    PID_Init(&CM3PID, 55, 0, 0, 8000); // 35   0.01
    PID_Init(&CM4PID, 55, 0, 0, 8000); // 35   0.01

    // 等待编码器接收以设定初始编码器偏差
    vTaskDelayUntil(&LastWakeTime, 1000);
    CM1_Encoder.ecdBias = Motor_Feedback.motor201Angle;
    CM2_Encoder.ecdBias = Motor_Feedback.motor202Angle;
    CM3_Encoder.ecdBias = Motor_Feedback.motor203Angle;
    CM4_Encoder.ecdBias = Motor_Feedback.motor204Angle;

    // 初始化编码器解算
    CAN_Get_Encoder_Bias(&CM1_Encoder);
    CAN_Get_Encoder_Bias(&CM2_Encoder);
    CAN_Get_Encoder_Bias(&CM3_Encoder);
    CAN_Get_Encoder_Bias(&CM4_Encoder);

    while (1) {
        // 安全模式
        if (DBusData.switchRight == 2) {
            Can_Set_CM_Current(CAN1, 0, 0, 0, 0);
            break;
        }

        // 解算编码器角度
        CAN_Update_Encoder_Data(&CM1_Encoder, Motor_Feedback.motor201Angle);
        CAN_Update_Encoder_Data(&CM2_Encoder, Motor_Feedback.motor202Angle);
        CAN_Update_Encoder_Data(&CM3_Encoder, Motor_Feedback.motor203Angle);
        CAN_Update_Encoder_Data(&CM4_Encoder, Motor_Feedback.motor204Angle);

        // 位置调整
        if (DBusData.switchLeft == 3) {
            targetFrontGroupOffset = targetFrontGroupPreset[groupMode] - CM3_Encoder.ecdAngle / 19.2;
            targetBackGroupOffset  = targetBackGroupPreset[groupMode] - CM2_Encoder.ecdAngle / 19.2;
            Can_Set_CM_Current(CAN1, 0, 0, 0, 0); //轮组电机断电
            lastSwitchLeft = 3;                   //更新模式信息
            vTaskDelayUntil(&LastWakeTime, 5);    //任务延时
            continue;
        }

        // 轮组模式判断
        if (DBusData.switchLeft == 1 && lastSwitchLeft == 3) {
            if (DBusData.switchRight == 1) {
                groupMode = 1;
            } else if (DBusData.switchRight == 3) {
                groupMode = 3;
            }
            lastSwitchLeft = 1;
        } else if (DBusData.switchLeft == 2 && lastSwitchLeft == 3) {
            if (DBusData.switchRight == 1) {
                groupMode = 2;

            } else if (DBusData.switchRight == 3) {
                groupMode = 4;
            }
            lastSwitchLeft = 2;
        }

        // 轮组模式解算
        if (groupMode <= 2) {
            targetFrontGroup = targetFrontGroupPreset[groupMode] - targetFrontGroupOffset;
            targetBackGroup  = targetBackGroupPreset[groupMode] - targetBackGroupOffset;
        } else {
            Can_Set_CM_Current(CAN1, -magic.value, magic.value, magic.value, -magic.value);
            vTaskDelayUntil(&LastWakeTime, 50);
            continue;
        }

        // PID解算
        PID_Calculate(&ChassisAnglePID1, -targetBackGroup, CM1_Encoder.ecdAngle / 19.2);
        PID_Calculate(&ChassisAnglePID2, targetBackGroup, CM2_Encoder.ecdAngle / 19.2);
        PID_Calculate(&ChassisAnglePID3, targetFrontGroup, CM3_Encoder.ecdAngle / 19.2);
        PID_Calculate(&ChassisAnglePID4, -targetFrontGroup, CM4_Encoder.ecdAngle / 19.2);

        PID_Calculate(&CM1PID, ChassisAnglePID1.output, Motor_Feedback.motor201Speed * kFeedback);
        PID_Calculate(&CM2PID, ChassisAnglePID2.output, Motor_Feedback.motor202Speed * kFeedback);
        PID_Calculate(&CM3PID, ChassisAnglePID3.output, Motor_Feedback.motor203Speed * kFeedback);
        PID_Calculate(&CM4PID, ChassisAnglePID4.output, Motor_Feedback.motor204Speed * kFeedback);

        // 输出电流
        Can_Set_CM_Current(CAN1, CM1PID.output, CM2PID.output, CM3PID.output, CM4PID.output);

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