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

    // 初始化角速度PID
    PID_Init(&PID_Chassis_Left, 1, 0, 0, 4000, 2000);
    PID_Init(&PID_Chassis_Right, 1, 0, 0, 4000, 2000);

    while (1) {

        // 计算输出电流PID
        PID_Calculate(&PID_Chassis_Left, (float) remoteData.lx, Motor_Chassis_Left.speed * RPM2RPS);
        PID_Calculate(&PID_Chassis_Right, (float) remoteData.lx, Motor_Chassis_Right.speed * RPM2RPS);

        // 输出电流值到电调
        Can_Send(CAN1, 0x200, PID_Chassis_Left.output, PID_Chassis_Right.output, 0, 0);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // debug
        DebugData.debug1 = remoteData.lx;
        DebugData.debug2 = PID_Chassis_Left.output * 1000;
        // DebugData.debug3 = ;
        // DebugData.debug4 = ;
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

    // Pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = -1 * Gyroscope_EulerData.pitch;
    float pitchAngleTargetRamp = 0;

    // 视觉系统
    int   lastSeq            = 0;
    float psYawAngleTarget   = 0;
    float psPitchAngleTarget = 0;

    // 初始化云台PID
    PID_Init(&PID_Stabilizer_Yaw_Angle, 16, 0, 0, 4000, 800); // i 0.06
    PID_Init(&PID_Stabilizer_Yaw_Speed, 14, 0, 0, 800, 2000);
    PID_Init(&PID_Stabilizer_Pitch_Angle, 20, 0.08, 0, 4000, 800);
    PID_Init(&PID_Stabilizer_Pitch_Speed, 8, 0, 0, 3000, 2000);

    while (1) {
        // 设置反馈
        yawAngle   = -1 * Gyroscope_EulerData.yaw;    // 逆时针为正
        yawSpeed   = -1 * ImuData.gz / GYROSCOPE_LSB; // 逆时针为正
        pitchAngle = -1 * Gyroscope_EulerData.pitch;  // 逆时针为正
        pitchSpeed = ImuData.gx / GYROSCOPE_LSB;      // 逆时针为正

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
        if (ABS(remoteData.rx) > 20) yawAngleTarget -= -1 * remoteData.rx / 660.0f * 180 * interval;
        if (ABS(remoteData.ry) > 20) pitchAngleTarget -= -1 * remoteData.ry / 660.0f * 150 * interval;
        yawAngleTarget += psYawAngleTarget;
        pitchAngleTarget += psPitchAngleTarget;
        MIAO(yawAngleTarget, -50, 50); //+-20
        MIAO(pitchAngleTarget, -20, 50);

        // 开机时pitch轴匀速抬起
        pitchAngleTargetRamp = RAMP(pitchRampStart, pitchAngleTarget, pitchRampProgress);
        if (pitchRampProgress < 1) {
            pitchRampProgress += 0.005f;
        }

        // 计算PID
        PID_Calculate(&PID_Stabilizer_Yaw_Angle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Stabilizer_Yaw_Speed, PID_Stabilizer_Yaw_Angle.output, yawSpeed);

        PID_Calculate(&PID_Stabilizer_Pitch_Angle, pitchAngleTargetRamp, pitchAngle);
        PID_Calculate(&PID_Stabilizer_Pitch_Speed, PID_Stabilizer_Pitch_Angle.output, pitchSpeed);

        // 输出电流
        Can_Send(CAN1, 0x200, 0, 0, PID_Stabilizer_Yaw_Speed.output, -1 * PID_Stabilizer_Pitch_Speed.output);

        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = ;
        // DebugData.debug2 = ;
        // DebugData.debug3 = ;
        // DebugData.debug4 = ;
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
        if (KTV_Play(Music_Soul)) break;
        vTaskDelayUntil(&LastWakeTime, 60);
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
    Magic_Init_Handle(&magic, 0); // 初始化调试数据的默认值
                                  // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500,
                                  // NULL, 6, NULL); xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send",
                                  // 500, NULL, 6,
                                  //                                                 NULL);
                                  // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6,
                                  // NULL); xTaskCreate(Task_Debug_Gyroscope_Sampling,
                                  // "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
