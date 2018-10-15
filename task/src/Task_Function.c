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
        PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
        PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
        PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

        // 输出电流值到电调
        Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}
/**
 * @brief 发射机构代码
 *
 * @param Parameters
 */
// Debug code For Jlink debug
uint8_t debugA = 0;
uint8_t debugB = 0;
uint8_t debugC = 0;
uint8_t debugD = 0;
uint8_t debugE = 0;
uint8_t debugF = 0;
uint8_t debugG = 0;
uint8_t debugH = 0;

void Task_Fire(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      rpm2rps      = 3.14 / 60;           // 转子的转速(round/min)换算成角速度(rad/s)
    // uint8_t    startCounter = 0;                   // 启动模式计数器

    // #define LASER_ON GPIO_SetBits(GPIOG, GPIO_Pin_13)    // 激光开启
    // #define LASER_OFF GPIO_ResetBits(GPIOG, GPIO_Pin_13) // 激光关闭

    // 标志位
    uint8_t frictState = 0;
    uint8_t stirState  = 0;
    uint8_t stirFlag   = 0;

    // 常量
    float frictSpeed   = -24 / 0.0595 * 2 * 60 / 2 / 3.14;
    float stirSpeedOne = 36 * 36;
    float stirSpeedTwo = 50 * 36;

    // 电机初始化
    Motor_Init(&Motor_LeftFrict, 1, 0);
    Motor_Init(&Motor_RightFrict, 1, 0);
    Motor_Init(&Motor_Stir, 1, 0);

    // PID 初始化
    PID_Init(&PID_LeftFrictSpeed, 20, 3, 0, 4000, 2000);
    PID_Init(&PID_RightFrictSpeed, 20, 3, 0, 4000, 2000);
    PID_Init(&PID_StirAnlge, 2, 0, 0, 4000, 2000);
    PID_Init(&PID_StirSpeed, 21, 0, 0, 4000, 2000);

    while (1) {
        // Debug Code
        if (remoteData.switchRight == 1) {
            frictState = 0;
            stirState  = 1;
        } else if (remoteData.switchRight == 3) {
            frictState = 0;
            stirState  = 0;
        }

        // 摩擦轮 PID 控制
        if (frictState == 0) {
            // LASER_OFF;                                                                          // 关闭激光
            PID_Increment_Calculate(&PID_LeftFrictSpeed, 0, Motor_LeftFrict.speed * rpm2rps);   // 左摩擦轮停止
            PID_Increment_Calculate(&PID_RightFrictSpeed, 0, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮停止
            Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, 0, 0);
        } else {
            // LASER_ON;                                                                          // 开启激光
            PID_Calculate(&PID_LeftFrictSpeed, frictSpeed, Motor_LeftFrict.speed * rpm2rps);   // 左摩擦轮转动
            PID_Calculate(&PID_RightFrictSpeed, frictSpeed, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮转动
            Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, 0, 0);
        }

        // 拨弹轮 PID 控制
        if (stirState == 0) { // 停止模式
            PID_Increment_Calculate(&PID_StirSpeed, 0, Motor_Stir.speed * rpm2rps);
            Can_Send(CAN1, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
        } else if (stirState == 1) { // 三连发模式
            // Debug Code
            // if (stirFlag < 3) {
            //     stirFlag++;
            //     PID_Increment_Calculate(&PID_StirAnlge, (Motor_Stir.angle - 36 * 60), Motor_Stir.angle);
            //     PID_Increment_Calculate(&PID_StirSpeed, PID_StirAnlge.output, Motor_Stir.speed);
            //     Can_Send(CAN1, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
            // } else {
            //     PID_Increment_Calculate(&PID_StirSpeed, 0, Motor_Stir.speed * rpm2rps);
            //     Can_Send(CAN1, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
            // }

            PID_Increment_Calculate(&PID_StirAnlge, (Motor_Stir.angle - 36 * 60), Motor_Stir.angle);
            PID_Increment_Calculate(&PID_StirSpeed, PID_StirAnlge.output, Motor_Stir.speed);
            Can_Send(CAN1, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
        } else if (stirState == 2) { // 连发模式
            PID_Increment_Calculate(&PID_StirSpeed, stirSpeedOne, Motor_Stir.speed * rpm2rps);
            Can_Send(CAN1, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
        }

        vTaskDelayUntil(&LastWakeTime, 5);
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
