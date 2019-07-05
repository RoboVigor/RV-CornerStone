/**
 * @brief 真*英雄
 * @version 1.2.0
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

    // 反馈值
    float yawAngle, yawSpeed, pitchAngle, pitchSpeed;

    // 目标值
    float yawAngleTarget   = 0;
    float pitchAngleTarget = 0;

    // Pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = Gyroscope_EulerData.roll;
    float pitchAngleTargetRamp = 0;

    // 视觉系统
    int   lastSeq            = 0;
    float psYawAngleTarget   = 0;
    float psPitchAngleTarget = 0;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 15, 0, 0, 3000, 0);
    PID_Init(&PID_Cloud_YawSpeed, 30, 0, 0, 6000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 15, 0.01, 0, 3000, 500);
    PID_Init(&PID_Cloud_PitchSpeed, 30, 0, 0, 6000, 0);

    while (1) {
        // 设置反馈
        yawAngle   = Gyroscope_EulerData.yaw;    // 逆时针为正
        yawSpeed   = ImuData.gz / GYROSCOPE_LSB; // 逆时针为正
        pitchAngle = Gyroscope_EulerData.roll;   // 逆时针为正
        pitchSpeed = ImuData.gy / GYROSCOPE_LSB; // 逆时针为正

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
        if (ABS(remoteData.rx) > 20) yawAngleTarget += -1 * remoteData.rx / 660.0f * 180 * interval;
        if (ABS(remoteData.ry) > 20) pitchAngleTarget += -1 * remoteData.ry / 660.0f * 150 * interval;
        // yawAngleTarget += psYawAngleTarget;
        // pitchAngleTarget += psPitchAngleTarget;
        // MIAO(yawAngleTarget, -50, 50); //+-20
        MIAO(pitchAngleTarget, -45, 10);

        // 开机时pitch轴匀速抬起
        pitchAngleTargetRamp = RAMP(pitchRampStart, pitchAngleTarget, pitchRampProgress);
        if (pitchRampProgress < 1) {
            pitchRampProgress += 0.005f;
        }

        // 计算PID
        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);

        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTargetRamp, pitchAngle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchSpeed);

        // 输出电流
        Can_Send(CAN1, 0x1FF, -1 * PID_Cloud_YawSpeed.output, PID_Cloud_PitchSpeed.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = yawAngleTarget;
        // DebugData.debug2 = yawAngle;
        // DebugData.debug3 = PID_Cloud_YawSpeed.output;
        // DebugData.debug4 = pitchAngleTargetRamp;
        // DebugData.debug5 = pitchAngle;
        // DebugData.debug6 = Gyroscope_EulerData.roll;
        // DebugData.debug6 = PID_Cloud_PitchSpeed.output;
        // DebugData.debug7 = pitchRampStart;
        // DebugData.debug8 = pitchAngleTarget;
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 底盘运动
    float vx = 0;
    float vy = 0;
    float vw = 0;
    float targetPower;

    // 反馈值
    float motorAngle, motorSpeed;
    float lastMotorAngle = Motor_Yaw.angle;
    float filter[6]      = {0, 0, 0, 0, 0, 0};
    int   filterp        = 0;
    float motorSpeedStable;

    // 底盘跟随PID
    float followDeadRegion = 3.0;
    PID_Init(&PID_Follow_Angle, 1, 0, 0, 1000, 0);
    PID_Init(&PID_Follow_Speed, 5, 0, 0, 1000, 0);

    // 麦轮速度PID
    PID_Init(&PID_LFCM, 28, 0, 0, 15000, 7500);
    PID_Init(&PID_LBCM, 28, 0, 0, 15000, 7500);
    PID_Init(&PID_RBCM, 28, 0, 0, 15000, 7500);
    PID_Init(&PID_RFCM, 28, 0, 0, 15000, 7500);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    while (1) {

        // 设置反馈值
        motorAngle     = Motor_Yaw.angle;                          // 电机角度
        motorSpeed     = (motorAngle - lastMotorAngle) / interval; // 电机角速度
        lastMotorAngle = motorAngle;                               // 保存为上一次的电机角度

        // 对电机角速度进行平均值滤波
        filter[filterp] = motorAngle;
        filterp         = filterp == 5 ? 0 : filterp + 1;
        int i;
        for (i = 0; i < 6; i++) {
            motorSpeedStable += filter[i];
        }
        motorSpeedStable = motorSpeedStable / 6.0f;

        // 根据运动模式计算PID
        PID_Calculate(&PID_Follow_Angle, 0, motorAngle);                             // 计算航向角角度PID
        PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output, motorSpeedStable); // 计算航向角角速度PID

        // 设置底盘总体移动速度
        vx = -remoteData.lx / 660.0f * 2; // 4
        vy = remoteData.ly / 660.0f * 6;  // 12
        vw = ABS(PID_Follow_Angle.error) < followDeadRegion ? 0 : (-1 * PID_Follow_Speed.output * DPS2RPS);

        // 麦轮解算及限速
        // targetPower = 80.0 - (60.0 - ChassisData.powerBuffer) / 60.0 * 80.0;
        Chassis_Update(&ChassisData, vx, vy, vw); // 麦轮解算
        // Chassis_Fix(&ChassisData, motorAngle);    // 修正旋转后底盘的前进方向
        Chassis_Limit_Rotor_Speed(&ChassisData, 550); // 设置转子速度上限 (rad/s)
        // Chassis_Limit_Power(&ChassisData, 80, targetPower, Judge.powerHeatData.chassis_power, interval); // 根据功率限幅

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调
        Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = ChassisData.referencePower * 1000;
        // DebugData.debug2 = ChassisData.power * 1000;
        // DebugData.debug3 = ChassisData.powerScale * 1000;
        // DebugData.debug4 = Motor_LF.speed * RPM2RPS;
        // DebugData.debug5 = ChassisData.powerBuffer;
        // DebugData.debug6 = targetPower;
        // DebugData.debug6 = PID_LFCM.output;
        // DebugData.debug7 = rotorSpeed[3];
        // DebugData.debug8 = rotorSpeed[3];
    }

    vTaskDelete(NULL);
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
void Task_Fire(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      rpm2rps      = 3.14 / 60;
    float      radius       = 0.0595;

    // 激光

    // 标志位
    uint8_t frictState = 1; // 摩擦轮标志位

    // 常量
    // float frictSpeed = -24 / 0.0595 * 2 * 60 / 2 / 3.14;  // 摩擦轮线速度(mps)转转速(rpm)

    // PID 初始化
    PID_Init(&PID_LeftFrictSpeed, 20, 0.05, 0, 10000, 5000);
    PID_Init(&PID_RightFrictSpeed, 20, 0.05, 0, 10000, 5000);
    PID_Init(&PID_Stir2006Speed, 25, 0, 0, 20000, 6000); //半径38
    // PID_Init(&PID_Stir3510Angle, 100, 0, 0, 15000, 10000); //半径110
    PID_Init(&PID_Stir3510Speed, 100, 10, 0, 15000, 10000);
    while (1) {
        // DebugCode 标志位设置

        // 2006,3510拨弹轮
        if (remoteData.switchRight == 3) {
            PID_Calculate(&PID_Stir3510Speed, 10, Motor_Stir3510.speed * rpm2rps);
            PID_Calculate(&PID_Stir2006Speed, 49, Motor_Stir2006.speed * rpm2rps);
            PID_Calculate(&PID_LeftFrictSpeed, -100, Motor_LeftFrict.speed * rpm2rps);  // 左摩擦轮
            PID_Calculate(&PID_RightFrictSpeed, 100, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮

            Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, PID_Stir2006Speed.output, PID_Stir3510Speed.output);
        } else if (remoteData.switchLeft == 1) {
            PID_LeftFrictSpeed.output_I  = 0;
            PID_RightFrictSpeed.output_I = 0;
            PID_Calculate(&PID_LeftFrictSpeed, 0, Motor_LeftFrict.speed * rpm2rps);   // 左摩擦轮停止
            PID_Calculate(&PID_RightFrictSpeed, 0, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮停止
            Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, 0, 0);
        }

        vTaskDelayUntil(&LastWakeTime, 10);
        // DebugData.debug1 = Motor_Stir3510.speed * rpm2rps;
        // DebugData.debug2 = PID_Stir3510Speed.output;
        // DebugData.debug3 = PID_Stir3510Speed.output_I;
        // DebugData.debug4 = Motor_LeftFrict.speed * rpm2rps;
        // DebugData.debug5 = pitchAngleTarget;
        // DebugData.debug6 = ImuData.gx;
        // DebugData.debug7 = ImuData.gy;
        // DebugData.debug8 = ImuData.gz;
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
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // TIM5CH1_CAPTURE_STA = 0;
    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    // xTaskCreate(Task_Fire, "Task_Fire", 400, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}