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

void Task_Mode_Switch(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    int        lastSwitchRight;
    int        lastSwitchLeft;
    int        modeChanged;
    delay_ms(1000);
    lastSwitchRight = remoteData.switchRight;
    lastSwitchLeft  = remoteData.switchLeft;
    sumsungMode     = 0;
    while (1) {
        modeChanged = 0;
        if (remoteData.switchLeft != lastSwitchLeft) {
            lastSwitchRight = remoteData.switchRight;
            lastSwitchLeft  = remoteData.switchLeft;
            modeChanged     = 1;
        }
        if (modeChanged == 1) {
            if (remoteData.switchLeft == 3) {
                sumsungMode = 0;
            } else if (remoteData.switchLeft == 1 && remoteData.switchRight == 1) {
                sumsungMode = 1;
            } else if (remoteData.switchLeft == 2 && remoteData.switchRight == 1) {
                sumsungMode = 2;
            } else if (remoteData.switchLeft == 1 && remoteData.switchRight == 3) {
                sumsungMode = 3;
            } else if (remoteData.switchLeft == 2 && remoteData.switchRight == 3) {
                sumsungMode = 8;
            }
        }
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

/**
 * @brief  底盘@三星轮组
 * @param  void *Parameters
 * @return void
 */
int  targetBackGroupOffset  = 0;
int  targetFrontGroupOffset = 0;
void Task_Sumsung(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      kFeedback    = 3.14 / 60;

    // 轮组变量
    int targetBackGroup  = 0;
    int targetFrontGroup = 0;

    // 硬编码轮组运动参数
    int targetFrontGroupPreset[] = {0, 950, 700}; // mode0,1,2,3
    int targetBackGroupPreset[]  = {0, 150, 800}; // mode0,1,2,3
    int currentAntiFall[]        = {200, 100};    // front,back

    // 角度PID
    PID_Init(&ChassisAnglePID1, 1.5, 0, 0, 100, 50); // 1.5
    PID_Init(&ChassisAnglePID2, 1.5, 0, 0, 100, 50); // 1.5
    PID_Init(&ChassisAnglePID3, 1.5, 0, 0, 100, 50); // 1.5
    PID_Init(&ChassisAnglePID4, 1.5, 0, 0, 100, 50); // 1.5

    // 速度PID 空载情况最佳状态p=35  下地状态p=55时大致可以驱动
    PID_Init(&CM1PID, 55, 0, 0, 8000, 4000); // 35   0.01
    PID_Init(&CM2PID, 55, 0, 0, 8000, 4000); // 35   0.01
    PID_Init(&CM3PID, 55, 0, 0, 8000, 4000); // 35   0.01
    PID_Init(&CM4PID, 55, 0, 0, 8000, 4000); // 35   0.01

    while (1) {
        targetFrontGroupPreset[2] = magic.value;

        // 位置调整
        if (sumsungMode == 0) {
            targetFrontGroupOffset = targetFrontGroup - Motor_SumsungRF.angle;
            targetBackGroupOffset  = targetBackGroup - Motor_SumsungRB.angle;
            Can_Send(CAN1, 0x200, 0, 0, 0, 0); //轮组电机断电
            vTaskDelayUntil(&LastWakeTime, 5); //任务延时
            continue;
        }

        // 轮组模式解算
        if (sumsungMode == 1 || sumsungMode == 2) {
            targetFrontGroup = targetFrontGroupPreset[sumsungMode] - targetFrontGroupOffset;
            targetBackGroup  = targetBackGroupPreset[sumsungMode] - targetBackGroupOffset;
        } else if (sumsungMode == 3) {
            targetFrontGroup = 0;
            targetBackGroup  = 0;
            Can_Send(CAN1, 0x200, -currentAntiFall[0], -currentAntiFall[1], currentAntiFall[1], currentAntiFall[0]); //增加阻力
            vTaskDelayUntil(&LastWakeTime, 5);
            continue;
        } else if (sumsungMode == 8) {
        }

        // 轮组PID解算
        PID_Calculate(&ChassisAnglePID1, -targetFrontGroup, Motor_SumsungLF.angle);
        PID_Calculate(&ChassisAnglePID2, -targetBackGroup, Motor_SumsungLB.angle);
        PID_Calculate(&ChassisAnglePID3, targetBackGroup, Motor_SumsungRB.angle);
        PID_Calculate(&ChassisAnglePID4, targetFrontGroup, Motor_SumsungRF.angle);

        PID_Calculate(&CM1PID, ChassisAnglePID1.output, Motor_SumsungLF.speed * kFeedback);
        PID_Calculate(&CM2PID, ChassisAnglePID2.output, Motor_SumsungLB.speed * kFeedback);
        PID_Calculate(&CM3PID, ChassisAnglePID3.output, Motor_SumsungRB.speed * kFeedback);
        PID_Calculate(&CM4PID, ChassisAnglePID4.output, Motor_SumsungRF.speed * kFeedback);

        // 输出电流
        Can_Send(CAN1, 0x200, CM1PID.output, CM2PID.output, CM3PID.output, CM4PID.output);

        vTaskDelayUntil(&LastWakeTime, 5);
    }

    vTaskDelete(NULL);
}
/**
 * @brief  底盘运动
 */
int  isTurning = 1;
int  wheelMode = 0;
void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    int        rotorSpeed[4];                      // 轮子转速
    float      rpm2rps        = 3.14 / 60;         // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    int        lastIsTurning  = 1;                 // 上一次的运动模式
    float      yawAngleTarget = 0;                 // 目标值
    float      yawAngleFeed, yawSpeedFeed;
    float      ChassisYSpeed;             // 底盘Y轴运动速度
    int        movingForwardSpeed  = 120; // 硬编码向前运动的转速
    int        movingBackwardSpeed = -80; // 硬编码向后运动的转速

    // 初始化麦轮角速度PID
    PID_Init(&PID_LBCM, 70, 0.5, 0, 6000, 3000);
    PID_Init(&PID_LFCM, 70, 0.5, 0, 6000, 3000);
    PID_Init(&PID_RBCM, 70, 0.5, 0, 6000, 3000);
    PID_Init(&PID_RFCM, 70, 0.5, 0, 6000, 3000);

    // 初始化航向角角度PID和角速度PID
    PID_Init(&PID_YawAngle, 10, 0, 0, 1000, 1000);
    PID_Init(&PID_YawSpeed, 2, 0, 0, 4000, 1000);

    while (1) {

        if (sumsungMode == 1 || sumsungMode == 2) {
            ChassisYSpeed = (float) movingForwardSpeed / 660.0;
        } else if (sumsungMode == 3) {
            ChassisYSpeed = (float) movingBackwardSpeed / 660.0;
        } else if (sumsungMode == 0) {
            ChassisYSpeed = (float) remoteData.ly / 660.0;
        }

        // 更新运动模式
        isTurning = ABS(remoteData.rx) < 5 && ABS(remoteData.ly) < 5 ? 0 : 1;

        // 设置反馈值
        yawAngleFeed = EulerAngle.Yaw;         // 航向角角度反馈
        yawSpeedFeed = mpu6500_data.gz / 16.4; // 航向角角速度反馈

        // 切换运动模式
        if (isTurning != lastIsTurning) {
            PID_YawAngle.output_I = 0;            // 清空角度PID积分
            PID_YawSpeed.output_I = 0;            // 清空角速度PID积分
            yawAngleTarget        = yawAngleFeed; // 更新角度PID目标值
            lastIsTurning         = isTurning;    // 更新lastMode
        }

        // 根据运动模式计算PID
        if (isTurning == 0) {
            PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngleFeed);      // 计算航向角角度PID
            PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeedFeed); // 计算航向角角速度PID
        } else {
            PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeedFeed); // 计算航向角角速度PID
        }

        // 设置底盘总体移动速度
        Chassis_Set_Speed((float) -remoteData.lx / 660.0, ChassisYSpeed, (float) PID_YawSpeed.output / 1320.0);

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Get_Rotor_Speed(rotorSpeed);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
        PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
        PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
        PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

        // 输出电流值到电调
        Can_Send(CAN2, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);
        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

/**
 * @brief  安全模式
 */
void Task_Safe_Mode(void *Parameters) {

    while (1) {
        if (remoteData.switchRight == 2) {
#if CAN1_ENABLED
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
#endif // CAN1_ENABLED
#if CAN2_ENABLED
            Can_Send(CAN2, 0x200, 0, 0, 0, 0);
#endif // CAN2_ENABLED
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }

    vTaskDelete(NULL);
}
