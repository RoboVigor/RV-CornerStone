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
 * @brief  底盘@三星轮组
 * @param  void *Parameters
 * @return void
 */
int  groupMode              = 0;
int  targetBackGroupOffset  = 0;
int  targetFrontGroupOffset = 0;
void Task_Sumsung(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      kFeedback    = 3.14 / 60;

    // 轮组变量
    int targetBackGroup  = 0;
    int targetFrontGroup = 0;
    int lastSwitchLeft   = 0;

    // 硬编码轮组运动参数
    int targetFrontGroupPreset[] = {0, 900, 700, 0}; // mode0,1,2,3
    int targetBackGroupPreset[]  = {0, 150, 750, 0}; // mode0,1,2,3
    int currentAntiFall[]        = {300, 500};       // front,back
    // int targetFrontGroupPreset[] = {0, 850, 700};
    // int targetBackGroupPreset[]  = {0, 150, 700};

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
        // 位置调整
        if (remoteData.switchLeft == 3) {
            targetFrontGroupOffset = targetFrontGroupPreset[groupMode] - Motor_SumsungRF.angle;
            targetBackGroupOffset  = targetBackGroupPreset[groupMode] - Motor_SumsungRB.angle;
            Can_Send(CAN1, 0x200, 0, 0, 0, 0); //轮组电机断电
            lastSwitchLeft = 3;                //更新模式信息
            vTaskDelayUntil(&LastWakeTime, 5); //任务延时
            continue;
        }

        // 轮组模式判断
        if (remoteData.switchLeft == 1 && lastSwitchLeft == 3) {
            if (remoteData.switchRight == 1) {
                groupMode = 1; //轮组模式1 前侧轮组上台阶
            } else if (remoteData.switchRight == 3) {
                groupMode = 3; //轮组模式3  三星轮组底盘向后下台阶
            }
            lastSwitchLeft = 1;
        } else if (remoteData.switchLeft == 2 && lastSwitchLeft == 3) {
            if (remoteData.switchRight == 1) {
                groupMode = 2; //轮组模式2 后侧轮组上台阶

            } else if (remoteData.switchRight == 3) {
                groupMode = 4; //轮组模式4 暂定
            }
            lastSwitchLeft = 2;
        }

        // 轮组模式解算
        if (groupMode <= 2) {
            targetFrontGroup = targetFrontGroupPreset[groupMode] - targetFrontGroupOffset;
            targetBackGroup  = targetBackGroupPreset[groupMode] - targetBackGroupOffset;
        } else if (groupMode == 3) {
            targetFrontGroupOffset = 0;
            targetBackGroupOffset  = 0;
            Can_Send(CAN1, 0x200, -currentAntiFall[0], -currentAntiFall[1], currentAntiFall[1], currentAntiFall[0]); //增加阻力
            vTaskDelayUntil(&LastWakeTime, 5);
            continue;
        } else if (groupMode == 4) {
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
int  wheelMode = 0;
void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    int        rotorSpeed[4];                      // 轮子转速
    float      rpm2rps        = 3.14 / 60;         // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    int        mode           = 2;                 // 底盘运动模式,1直线,2转弯
    int        lastMode       = 2;                 // 上一次的运动模式
    float      yawAngleTarget = 0;                 // 目标值
    float      yawAngleFeed, yawSpeedFeed;
    int        target = 35; // 麦轮目标值

    // 初始化麦轮角速度PID
    PID_Init(&PID_LBCM, 70, 0.5, 0, 6000, 3000);
    PID_Init(&PID_LFCM, 70, 0.5, 0, 6000, 3000);
    PID_Init(&PID_RBCM, 70, 0.5, 0, 6000, 3000);
    PID_Init(&PID_RFCM, 70, 0.5, 0, 6000, 3000);

    // 初始化航向角角度PID和角速度PID
    PID_Init(&PID_YawAngle, 10, 0, 0, 1000, 1000);
    PID_Init(&PID_YawSpeed, 2, 0, 0, 4000, 1000);

    while (1) {
        //麦轮模式切换
        if (remoteData.switchLeft == 3) {
            if (remoteData.switchRight == 1) {
                wheelMode = 1; //模式1 摇杆控制麦轮
            } else if (remoteData.switchRight == 3) {
                wheelMode = 2; //模式2 麦轮控制底盘 整体向后运动
            }
        } else {
            wheelMode = 3; //模式3 麦轮控制底盘  整体向前运动(伴随三星轮组同步进行)
        }

        if (wheelMode == 1) {
            // 更新运动模式
            mode = ABS(remoteData.rx) < 5 ? 1 : 2;

            // 设置反馈值
            yawAngleFeed = EulerAngle.Yaw;         // 航向角角度反馈
            yawSpeedFeed = mpu6500_data.gz / 16.4; // 航向角角速度反馈

            // 切换运动模式
            if (mode != lastMode) {
                PID_YawAngle.output_I = 0;            // 清空角度PID积分
                PID_YawSpeed.output_I = 0;            // 清空角速度PID积分
                yawAngleTarget        = yawAngleFeed; // 更新角度PID目标值
                lastMode              = mode;         // 更新lastMode
            }

            // 根据运动模式计算PID
            if (mode == 1) {
                PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngleFeed);      // 计算航向角角度PID
                PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeedFeed); // 计算航向角角速度PID
            } else {
                PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeedFeed); // 计算航向角角速度PID
            }

            // 设置底盘总体移动速度
            Chassis_Set_Speed((float) -remoteData.lx / 660.0, (float) remoteData.ly / 660.0, (float) PID_YawSpeed.output / 1320.0);

            // 麦轮解算&限幅,获得轮子转速
            Chassis_Get_Rotor_Speed(rotorSpeed);

            // 计算输出电流PID
            PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
            PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
            PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
            PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

            // 输出电流值到电调
            Can_Send(CAN2, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);
        }

        if (wheelMode == 2) {
            PID_Calculate(&PID_LFCM, -target, Motor_LF.speed * rpm2rps);
            PID_Calculate(&PID_LBCM, -target, Motor_LB.speed * rpm2rps);
            PID_Calculate(&PID_RBCM, target, Motor_RB.speed * rpm2rps);
            PID_Calculate(&PID_RFCM, target, Motor_RF.speed * rpm2rps);

            Can_Send(CAN2, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);
        }

        if (wheelMode == 3) {
            PID_Calculate(&PID_LFCM, target, Motor_LF.speed * rpm2rps);
            PID_Calculate(&PID_LBCM, target, Motor_LB.speed * rpm2rps);
            PID_Calculate(&PID_RBCM, -target, Motor_RB.speed * rpm2rps);
            PID_Calculate(&PID_RFCM, -target, Motor_RF.speed * rpm2rps);
            Can_Send(CAN2, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);
        }
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
