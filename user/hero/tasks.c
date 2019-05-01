/**
 * @brief 甩锅小车
 * @version 0.8.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}

void Task_Gimbal(void *Parameters) {

    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 单位换算
    float rpm2rps = 2 * 3.1415926f / 60.0f; // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    float dps2rps = 3.1415926f / 180.0f;    // degree per second to rad per second
    float rps2dps = 180.0f / 3.1415926f;    // degree per second to rad per second

    // 反馈值
    float motorAngle, motorSpeed, yawAngle, yawSpeed, pitchAngle, pitchSpeed;

    // 目标值
    float yawAngleTarget   = 0;
    float pitchAngleTarget = 0;

    // pitch ramp
    float pitchRampProgress = 0;
    float pitchRampStart    = Gyroscope_EulerData.pitch;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 1, 0, 0, 16000, 0);
    PID_Init(&PID_Cloud_YawSpeed, 1, 0, 0, 16000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 1, 0, 0, 16000, 0);
    PID_Init(&PID_Cloud_PitchSpeed, 1, 0, 0, 16000, 0);

    while (1) {
        // 设置反馈值
        motorAngle = Motor_Yaw.angle;                                                  // 电机角度
        motorSpeed = (PID_Follow_Angle.error - PID_Follow_Angle.lastError) / interval; // 电机角速度 todo:验证这样写行不行
        yawAngle   = Gyroscope_EulerData.yaw;
        yawSpeed   = ImuData.gz * rps2dps;
        pitchAngle = Gyroscope_EulerData.pitch;
        pitchSpeed = ImuData.gx * rps2dps;

        // 设置目标
        yawAngleTarget += (float) remoteData.rx / 660.0f * 30 * interval;
        pitchAngleTarget += (float) remoteData.ry / 660.0f * 20 * interval;
        MIAO(yawAngleTarget, -30, 30);
        MIAO(pitchAngleTarget, -50, 5);
        pitchAngleTarget = RAMP(pitchRampStart, pitchAngleTarget, pitchRampProgress);
        if (pitchRampProgress < 1) {
            pitchRampProgress += 0.005f;
        }

        // 计算PID
        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output / interval, yawSpeed);

        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTarget, pitchAngle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output / interval, pitchSpeed);

        // 输出电流
        // Can_Send(CAN1, 0x1FF, PID_Cloud_PitchSpeed.output, PID_Cloud_YawAngle.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        DebugData.debug1 = ImuData.gx * rps2dps;
        DebugData.debug2 = pitchSpeed;
        // DebugData.debug1 = PID_Cloud_YawAngle.target;
        // DebugData.debug2 = PID_Cloud_YawAngle.feedback;
        DebugData.debug3 = PID_Cloud_YawSpeed.target;
        DebugData.debug4 = PID_Cloud_YawSpeed.feedback;
        DebugData.debug5 = PID_Cloud_PitchAngle.target;
        DebugData.debug6 = PID_Cloud_PitchAngle.feedback;
        DebugData.debug7 = PID_Cloud_PitchSpeed.target;
        DebugData.debug8 = PID_Cloud_PitchSpeed.feedback;
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 单位换算
    float rpm2rps = 2 * 3.1415926f / 60.0f; // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    float dps2rps = 3.1415926f / 180.0f;    // degree per second to rad per second
    float rps2dps = 180.0f / 3.1415926f;    // degree per second to rad per second

    // 底盘运动
    float vx = 0;
    float vy = 0;
    float vw = 0;
    int   rotorSpeed[4];  // 转子转速
    float powerScale = 1; // 功率系数

    // 反馈值
    float motorAngle, motorSpeed, yawAngle, yawSpeed;

    // 底盘跟随PID
    PID_Init(&PID_Follow_Angle, 1, 0, 0, 1000, 0);
    PID_Init(&PID_Follow_Speed, 1, 0, 0, 1000, 0);

    // 麦轮速度PID
    PID_Init(&PID_LFCM, 28, 0, 0, 3500, 1750);
    PID_Init(&PID_LBCM, 28, 0, 0, 3500, 1750);
    PID_Init(&PID_RBCM, 28, 0, 0, 3500, 1750);
    PID_Init(&PID_RFCM, 28, 0, 0, 3500, 1750);

    //功率PID
    PID_Init(&PID_Power, 0.5, 0.02, 0, 1000, 500);

    while (1) {

        // 设置反馈值
        motorAngle = Motor_Yaw.angle;                                                  // 电机角度
        motorSpeed = (PID_Follow_Angle.error - PID_Follow_Angle.lastError) / interval; // 电机角速度 todo:验证这样写行不行
        yawAngle   = Gyroscope_EulerData.yaw;
        yawSpeed   = ImuData.gz * rps2dps;

        // 根据运动模式计算PID
        PID_Calculate(&PID_Follow_Angle, 0, motorAngle);                   // 计算航向角角度PID
        PID_Calculate(&PID_Follow_Speed, PID_YawAngle.output, motorSpeed); // 计算航向角角速度PID

        // 设置底盘总体移动速度
        vx = -remoteData.lx / 660.0f;
        vy = remoteData.ly / 660.0f * 3;
        vw = (float) PID_Follow_Speed.output * dps2rps;
        Chassis_Update(&ChassisData, vx, vy, vw);

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Get_Rotor_Speed(&ChassisData, rotorSpeed);

        // 根据功率限幅
        // powerScale    = Chassis_Power_Control(lx, ly);
        rotorSpeed[0] = rotorSpeed[0] * powerScale;
        rotorSpeed[1] = rotorSpeed[1] * powerScale;
        rotorSpeed[2] = rotorSpeed[2] * powerScale;
        rotorSpeed[3] = rotorSpeed[3] * powerScale;

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
        PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
        PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
        PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

        // 输出电流值到电调
        // Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = PID_Follow_Angle.target;
        // DebugData.debug2 = PID_Follow_Angle.feedback;
        // DebugData.debug3 = PID_Follow_Angle.output;
        // DebugData.debug4 = PID_Follow_Speed.target;
        // DebugData.debug5 = PID_Follow_Speed.feedback;
        // DebugData.debug6 = PID_Follow_Speed.output;
        // DebugData.debug7 = PID_Follow_Speed.output;
        // DebugData.debug8 = PID_Follow_Speed.output;
    }

    vTaskDelete(NULL);
}

float Chassis_Power_Control(float VX, float VY) {

    powerfeed = Judge.powerHeatData.chassisPower;
    if (powerfeed >= 60) {
        PID_Calculate(&PID_Power, 700, powerfeed * 10);

        return (float) (PID_Power.output + 1000) / 1000.0;
    } else {
        if (VX == 0 && VY == 0) {
            PID_Power.output_I = 0;
            PID_Power.output   = 0;
        }
        return 1.0f;
    }
}

void Task_Debug_Magic_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        // printf();
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 500);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 发射机构代码
 *
 * @param Parameters
 */

int turnNumber = 1;

void Task_Fire(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      rpm2rps      = 3.14 / 60;           // 转子的转速(round/min)换算成角速度(rad/s)
    float      r            = 0.0595;
    // uint8_t    startCounter = 0;                   // 启动模式计数器

    // 标志位
    uint8_t frictState = 0;
    uint8_t stirState  = 0;
    uint8_t stirFlag   = 0;
    int     lastSwitch = 0;

    // 常量
    // float frictSpeed = -24 / 0.0595 * 2 * 60 / 2 / 3.14;
    // 摩擦轮线速度(mps)转转速(rpm)
    float frictSpeed = 336; // 336 rad/s
    float stirSpeed  = -40;

    // PID 初始化
    PID_Init(&PID_LeftFrictSpeed, 25, 0.5, 0, 20000, 6000);  // 4 0.08 0    6000  1000
    PID_Init(&PID_RightFrictSpeed, 25, 0.5, 0, 20000, 6000); // 10 0.08           1000
    PID_Init(&PID_StirAnlge, 12, 0.01, 0, 4000, 2000);       // 12 0.01
    PID_Init(&PID_StirSpeed, 5, 0.2, 0, 4000, 2000);         // 5

    while (1) {
        if (remoteData.switchRight == 1) {
            frictState = 1;
        } else if (remoteData.switchRight == 3) {
            frictState = 0;
        }

        // 摩擦轮 PID 控制
        if (frictState == 0) {
            LASER_OFF;                                                                              // 关闭激光
            PID_Increment_Calculate(&PID_LeftFrictSpeed, 0, Motor_LeftFrict.speed * rpm2rps * r);   // 左摩擦轮停止
            PID_Increment_Calculate(&PID_RightFrictSpeed, 0, Motor_RightFrict.speed * rpm2rps * r); // 右摩擦轮停止
            PID_LeftFrictSpeed.output  = PID_LeftFrictSpeed.output / 0.0595 * 60 / 3.14;
            PID_RightFrictSpeed.output = PID_RightFrictSpeed.output / 0.0595 * 60 / 3.14;
            Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, 0, 0);
        } else {
            LASER_ON;                                                                           // 开启激光
            PID_Calculate(&PID_LeftFrictSpeed, frictSpeed, Motor_LeftFrict.speed * rpm2rps);    // 左摩擦轮转动
            PID_Calculate(&PID_RightFrictSpeed, -frictSpeed, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮转动
            Can_Send(CAN2, 0x200, PID_RightFrictSpeed.output, PID_LeftFrictSpeed.output, 0, 0);
        }
        if (remoteData.switchLeft == 1) {
            // TIM_SetCompare1(TIM4, 15);
        }
        if (remoteData.switchLeft == 2) {
            stirState = 2;
        } else if (remoteData.switchLeft == 3) {
            // lastSwitch = 3;
            stirState = 0;
            // TIM_SetCompare1(TIM4, 7);
        }

        //拨弹轮 PID 控制
        if (stirState == 0) { // 停止模式
            PID_Increment_Calculate(&PID_StirSpeed, 0, Motor_Stir.speed * rpm2rps);
        } else if (stirState == 2) { // 连发模式
            PID_Increment_Calculate(&PID_StirSpeed, stirSpeed, Motor_Stir.speed * rpm2rps);
        }
        Can_Send(CAN2, 0x1FF, 0, 0, PID_StirSpeed.output, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

    // 初始化全局变量
    Handle_Init();

    // BSP们
    BSP_GPIO_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_DMA_Init();
    BSP_TIM_Init();
    BSP_NVIC_Init();
    BSP_USER_Init();

    // 初始化陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData);

    // 调试任务
#if DEBUG_ENABLED
    // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    // xTaskCreate(Task_Fire, "Task_Fire", 400, NULL, 6, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}