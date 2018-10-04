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
    int target           = 0;
    int targetBackGroup  = 0;
    int targetFrontGroup = 0;
    int lastSwitchLeft   = 0;

    // 硬编码轮组运动参数
    int targetFrontGroupPreset[] = {0, 700, 600, 700, 0};
    int targetBackGroupPreset[]  = {0, 100, 600, 100, 0};

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

    // 等待编码器接收以设定初始编码器偏差
    vTaskDelayUntil(&LastWakeTime, 1000);
    // Encoder_SumsungLB.ecdBias = Motor_SumSungLB.angle;
    // Encoder_SumsungRB.ecdBias = Motor_SumSungRB.angle;
    // Encoder_SumsungRF.ecdBias = Motor_SumSungRF.angle;
    // Encoder_SumsungLF.ecdBias = Motor_SumSungLF.angle;

    // 初始化编码器解算
    Motor_Init(&Encoder_SumsungLB, Motor_SumSungLB.position);
    Motor_Init(&Encoder_SumsungRB, Motor_SumSungRB.position);
    Motor_Init(&Encoder_SumsungRF, Motor_SumSungRF.position);
    Motor_Init(&Encoder_SumsungLF, Motor_SumSungLF.position);

    while (1) {
        // // 安全模式
        // if (remoteData.switchRight == 2) {
        //     Can_Send(CAN1, 0x200, 0, 0, 0, 0);
        //     break;
        // }

        // 解算编码器角度
        Motor_Update(&Encoder_SumsungLB, Motor_SumSungLB.position);
        Motor_Update(&Encoder_SumsungRB, Motor_SumSungRB.position);
        Motor_Update(&Encoder_SumsungRF, Motor_SumSungRF.position);
        Motor_Update(&Encoder_SumsungLF, Motor_SumSungLF.position);

        // 位置调整
        if (remoteData.switchLeft == 3) {
            targetFrontGroupOffset = targetFrontGroupPreset[groupMode] - Encoder_SumsungRF.angle / 19.2;
            targetBackGroupOffset  = targetBackGroupPreset[groupMode] - Encoder_SumsungRB.angle / 19.2;
            Can_Send(CAN1, 0x200, 0, 0, 0, 0); //轮组电机断电
            lastSwitchLeft = 3;                //更新模式信息
            vTaskDelayUntil(&LastWakeTime, 5); //任务延时
            continue;
        }

        // 轮组模式判断
        if (remoteData.switchLeft == 1 && lastSwitchLeft == 3) {
            if (remoteData.switchRight == 1) {
                groupMode = 1;
            } else if (remoteData.switchRight == 3) {
                groupMode = 3;
            }
            lastSwitchLeft = 1;
        } else if (remoteData.switchLeft == 2 && lastSwitchLeft == 3) {
            if (remoteData.switchRight == 1) {
                groupMode = 2;

            } else if (remoteData.switchRight == 3) {
                groupMode = 4;
            }
            lastSwitchLeft = 2;
        }

        // 轮组模式解算
        if (groupMode <= 2) {
            targetFrontGroup = targetFrontGroupPreset[groupMode] - targetFrontGroupOffset;
            targetBackGroup  = targetBackGroupPreset[groupMode] - targetBackGroupOffset;

        } else {
            Can_Send(CAN1, 0x200, -300, 300, 300, -300);
            vTaskDelayUntil(&LastWakeTime, 50);
            continue;
        }

        // 轮组PID解算
        PID_Calculate(&ChassisAnglePID1, -targetBackGroup, Encoder_SumsungLB.angle / 19.2);
        PID_Calculate(&ChassisAnglePID2, targetBackGroup, Encoder_SumsungRB.angle / 19.2);
        PID_Calculate(&ChassisAnglePID3, targetFrontGroup, Encoder_SumsungRF.angle / 19.2);
        PID_Calculate(&ChassisAnglePID4, -targetFrontGroup, Encoder_SumsungLF.angle / 19.2);

        PID_Calculate(&CM1PID, ChassisAnglePID1.output, Motor_SumSungLB.speed * kFeedback);
        PID_Calculate(&CM2PID, ChassisAnglePID2.output, Motor_SumSungRB.speed * kFeedback);
        PID_Calculate(&CM3PID, ChassisAnglePID3.output, Motor_SumSungRF.speed * kFeedback);
        PID_Calculate(&CM4PID, ChassisAnglePID4.output, Motor_SumSungLF.speed * kFeedback);

        // 输出电流
        Can_Send(CAN1, 0x200, CM1PID.output, CM2PID.output, CM3PID.output, CM4PID.output);

        vTaskDelayUntil(&LastWakeTime, 5);
    }

    vTaskDelete(NULL);
}
/**
 * @brief  底盘运动
 */

void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    int        rotorSpeed[4];                      // 轮子转速
    float      rpm2rps        = 3.14 / 60;         // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    int        mode           = 2;                 // 底盘运动模式,1直线,2转弯
    int        lastMode       = 2;                 // 上一次的运动模式
    float      yawAngleTarget = 0;                 // 目标值
    float      yawAngleFeed, yawSpeedFeed;         // 反馈值

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_LBCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_RBCM, 15, 0.3, 0, 4000, 2000);
    PID_Init(&PID_RFCM, 15, 0.3, 0, 4000, 2000);

    // 初始化航向角角度PID和角速度PID
    PID_Init(&PID_YawAngle, 10, 0, 0, 1000, 1000);
    PID_Init(&PID_YawSpeed, 2, 0, 0, 4000, 1000);

    while (1) {

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
        Chassis_Set_Speed((float) remoteData.lx / 660.0, (float) -remoteData.ly / 660.0, (float) PID_YawSpeed.output / 1320.0);

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Get_Rotor_Speed(rotorSpeed);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
        PID_Calculate(&PID_LBCM, rotorSpeed[3], Motor_LB.speed * rpm2rps);
        PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
        PID_Calculate(&PID_RFCM, rotorSpeed[1], Motor_RF.speed * rpm2rps);

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
#endif // CAN1_ENABLED
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }

    vTaskDelete(NULL);
}
