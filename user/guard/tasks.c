/**
 * @brief 哨兵
 * @version 0.8.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 目标值
    float leftTarget  = 0;
    float rightTarget = 0;

    // 初始化角速度PID
    PID_Init(&PID_Chassis_Left, 2, 0, 0, 4000, 2000);  // 3 0 0
    PID_Init(&PID_Chassis_Right, 2, 0, 0, 4000, 2000); // 3 0 0

    while (1) {

        // 设置目标值
        if (ABS(remoteData.lx) > 20) leftTarget = -1 * remoteData.lx / 660.0f * 500;
        if (ABS(remoteData.lx) > 20) rightTarget = -1 * remoteData.lx / 660.0f * 500;

        // 计算输出电流PID
        PID_Calculate(&PID_Chassis_Left, leftTarget, Motor_Chassis_Left.speed * RPM2RPS);
        PID_Calculate(&PID_Chassis_Right, rightTarget, Motor_Chassis_Right.speed * RPM2RPS);

        // 输出电流值到电调
        Can_Send(CAN1, 0x200, PID_Chassis_Left.output, PID_Chassis_Right.output, PID_Stabilizer_Yaw_Speed.output, PID_Stabilizer_Pitch_Speed.output);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // debug
        // DebugData.debug1 = remoteData.lx;
        // DebugData.debug2 = PID_Chassis_Left.feedback;
        // DebugData.debug3 = PID_Chassis_Left.feedback - remoteData.lx;
    }

    vTaskDelete(NULL);
}
void Task_Gimbal(void *Parameters) {

    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 反馈值
    float yawAngle, yawSpeed, pitchAngle, pitchSpeed;

    // 目标值
    float yawAngleTarget   = 0;
    float pitchAngleTarget = 0;

    // 视觉系统
    int   lastSeq            = 0;
    float psYawAngleTarget   = 0;
    float psPitchAngleTarget = 0;

    // 初始化云台PID
    PID_Init(&PID_Stabilizer_Yaw_Angle, 15, 0, 0, 16000, 0);
    PID_Init(&PID_Stabilizer_Yaw_Speed, 30, 0, 0, 4000, 0);
    PID_Init(&PID_Stabilizer_Pitch_Angle, 20, 0, 0, 16000, 0);
    PID_Init(&PID_Stabilizer_Pitch_Speed, 40, 0, 0, 4000, 0);

    while (1) {
        // 设置反馈
        yawAngle   = Motor_Stabilizer_Yaw.angle;
        yawSpeed   = Motor_Stabilizer_Yaw.speed * RPM2RPS;
        pitchAngle = Motor_Stabilizer_Pitch.angle;
        pitchSpeed = Motor_Stabilizer_Pitch.speed * RPM2RPS;

        // 视觉系统
        // if (lastSeq != Ps.seq) {
        //     lastSeq          = Ps.seq;
        //     psYawAngleTarget = Ps.gimbalAimData.yaw_angle_diff;
        //     psPitchAngleTarget += Ps.gimbalAimData.pitch_angle_diff;
        //     MIAO(psYawAngleTarget, -5, 5);
        //     MIAO(psPitchAngleTarget, -5, 5);
        // } else {
        //     psYawAngleTarget   = 0;
        //     psPitchAngleTarget = 0;
        // }

        // 设置角度目标
        if (ABS(remoteData.rx) > 20) yawAngleTarget -= 1 * remoteData.rx / 660.0f * 180 * interval;
        if (ABS(remoteData.ry) > 20) pitchAngleTarget += remoteData.ry / 660.0f * 150 * interval;
        yawAngleTarget += psYawAngleTarget;
        pitchAngleTarget += psPitchAngleTarget;
        // MIAO(yawAngleTarget, -20, 20);
        MIAO(pitchAngleTarget, 0, 95);

        // 计算PID
        PID_Calculate(&PID_Stabilizer_Yaw_Angle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Stabilizer_Yaw_Speed, PID_Stabilizer_Yaw_Angle.output, yawSpeed);

        PID_Calculate(&PID_Stabilizer_Pitch_Angle, pitchAngleTarget, pitchAngle);
        PID_Calculate(&PID_Stabilizer_Pitch_Speed, PID_Stabilizer_Pitch_Angle.output, pitchSpeed);

        // 输出电流
        Can_Send(CAN1, 0x200, PID_Chassis_Left.output, PID_Chassis_Right.output, PID_Stabilizer_Yaw_Speed.output, PID_Stabilizer_Pitch_Speed.output);

        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = PID_Stabilizer_Pitch_Speed.feedback;
        // DebugData.debug2 = PID_Stabilizer_Pitch_Angle.output;
        // DebugData.debug3 = PID_Stabilizer_Pitch_Angle.feedback;
        // DebugData.debug4 = pitchAngleTarget;
        // DebugData.debug5 = PID_Stabilizer_Pitch_Angle.feedback - pitchAngleTarget;
        // DebugData.debug6 = Motor_Stabilizer_Pitch.position;
    }
    vTaskDelete(NULL);
}

void Task_Debug_Magic_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        // printf("Yaw: %f \r\n", Gyroscope_EulerData.yaw);
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 500);
    }
    vTaskDelete(NULL);
}

void Task_Stir(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟

    // 目标值
    float stirAngleTarget = 0;

    // PID 初始化
    PID_Init(&PID_Stir_Angle, 12, 0.01, 0, 4000, 2000); // 12 0.01
    PID_Init(&PID_Stir_Speed, 5, 0.2, 0, 4000, 2000);   // 5

    while (1) {

        // 设置目标值
        stirAngleTarget += 60.0f;

        // 计算PID
        PID_Calculate(&PID_Stir_Angle, stirAngleTarget, Motor_Stir.angle);
        PID_Calculate(&PID_Stir_Speed, PID_Stir_Angle.output, Motor_Stir.speed * RPM2RPS);

        // 输出电流
        Can_Send(CAN1, 0x1FF, 0, 0, -1 * PID_Stir_Speed.output, 0);

        vTaskDelayUntil(&LastWakeTime, 5000);

        // 调试信息
        DebugData.debug1 = PID_Stir_Speed.feedback;
        DebugData.debug2 = PID_Stir_Angle.output;
        DebugData.debug3 = PID_Stir_Angle.feedback;
        DebugData.debug4 = stirAngleTarget;
    }

    vTaskDelete(NULL);
}

void Task_Frict(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      r            = 0.0595;
    // uint8_t    startCounter = 0;                   // 启动模式计数器

    // 标志位
    uint8_t frictState = 1;

    // 常量
    // float frictSpeed = -24 / 0.0595 * 2 * 60 / 2 / 3.14;
    // 摩擦轮线速度(mps)转转速(rpm)
    float frictSpeed = 336; // 336 rad/s

    // PID 初始化
    PID_Init(&PID_LeftFrict, 25, 0.5, 0, 20000, 6000);  // 4 0.08 0    6000  1000
    PID_Init(&PID_RightFrict, 25, 0.5, 0, 20000, 6000); // 10 0.08           1000

    while (1) {

        // 摩擦轮 PID 控制
        // LASER_ON;                                                                        // 开启激光
        PID_Calculate(&PID_LeftFrict, frictSpeed, Motor_LeftFrict.speed * RPM2RPS);    // 左摩擦轮转动
        PID_Calculate(&PID_RightFrict, -frictSpeed, Motor_RightFrict.speed * RPM2RPS); // 右摩擦轮转动
        Can_Send(CAN2, 0x200, PID_RightFrict.output, PID_LeftFrict.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        LED_Run_Horse_XP(); // XP开机动画,建议延时200ms
        // LED_Run_Horse(); // 跑马灯,建议延时20ms
        vTaskDelayUntil(&LastWakeTime, 200);
    }

    vTaskDelete(NULL);
}

void Task_Startup_Music(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        if (KTV_Play(Music_Earth)) break;
        vTaskDelayUntil(&LastWakeTime, 120);
    }
    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

    // 初始化全局变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 初始化陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData);

    // 调试任务
#if DEBUG_ENABLED
    // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6,
    // NULL);
#endif

    // 低级任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 运动控制任务
    // xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    // xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    xTaskCreate(Task_Frict, "Task_Frict", 400, NULL, 6, NULL);
    xTaskCreate(Task_Stir, "Task_Stir", 400, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}