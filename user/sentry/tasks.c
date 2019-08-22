/**
 * @brief 哨兵
 * @version 1.2.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (SafetyMode) {
            vTaskSuspendAll();
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            Can_Send(CAN1, 0x1ff, 0, 0, 0, 0);
            PWM_Set_Compare(&PWM_Snail1, 0.376 * 1250);
            PWM_Set_Compare(&PWM_Snail2, 0.376 * 1250);
        }
        vTaskDelay(5);
    }
    vTaskDelete(NULL);
}

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        FrictEnabled = remoteData.switchLeft == 3;
        LaserEnabled = remoteData.switchLeft == 3;
        // StirEnabled  = (remoteData.switchLeft == 3) && (remoteData.switchRight == 1);
        StirEnabled = remoteData.switchRight == 1;
        PsEnabled   = remoteData.switchLeft == 2;
        AutoMode    = (remoteData.switchLeft == 2) && (remoteData.switchRight == 1);
        SafetyMode  = remoteData.switchRight == 2;

        if ((remoteData.switchLeft == 1 && remoteData.switchRight == 1) || (!remoteData.state)) {
            FrictEnabled = 1;
            LaserEnabled = 0;
            StirEnabled  = 0;
            PsEnabled    = 1;
            AutoMode     = 1;
        }

        vTaskDelayUntil(&LastWakeTime, 5);
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

    // 视觉系统
    int lastSeq          = 0;
    int visionCounter    = 0;
    int maxVisionTimeout = 500 / intervalms;

    // 挨打
    int lastBlood      = 600;
    int hurtCounter    = 0;
    int maxHurtTimeout = 5000 / intervalms;

    // 自动运动
    uint16_t random         = 0;
    int      lastMotionType = 1;
    int      motionType     = 1;
    int      lastAutoMode   = 0;
    int      direction      = 1;
    float    timer          = 0.0;
    float    maxTime;
    float    minTime;
    int      maxSpeed;
    int      minSpeed;

    // 光电开关
    int optoelectronicDirection = 0;

    // 功率限制
    float power;
    float powerBuffer;
    float targetPower;

    // 初始化角速度PID
    PID_Init(&PID_Chassis_Left, 30, 0, 0, 4000, 2000);
    PID_Init(&PID_Chassis_Right, 30, 0, 0, 4000, 2000);

    // 初始化底盘
    Chassis_Init(&ChassisData);
    ChassisData.maxPower       = 20;
    ChassisData.maxPowerBuffer = 200;

    while (1) {
        // 视觉系统
        if (!PsEnabled) {
            lastSeq = Ps.autoaimData.seq;
            visionCounter++;
        } else if (lastSeq != Ps.autoaimData.seq) {
            lastSeq = Ps.autoaimData.seq;
            if (Ps.autoaimData.yaw_angle_diff == 0 && Ps.autoaimData.pitch_angle_diff == 0 && Ps.autoaimData.biu_biu_state == 0) {
                visionCounter++;
            } else {
                visionCounter = 0;
                motionType    = 2;
            }
        } else {
            visionCounter++;
        }

        // 挨打
        if (Judge.robotState.remain_HP == lastBlood) {
            hurtCounter++;
        } else {
            hurtCounter = 0;
            motionType  = 2;
        }
        lastBlood = Judge.robotState.remain_HP;

        // 巡逻
        if ((visionCounter >= maxVisionTimeout) && (hurtCounter >= maxHurtTimeout)) {
            motionType = 1;
        }
        if (visionCounter == INT_MAX) {
            visionCounter = maxVisionTimeout;
        }
        if (hurtCounter == INT_MAX) {
            hurtCounter = maxHurtTimeout;
        }

        // 随机数生成
        srand(Motor_Chassis_Left.position);
        random = rand();

        // 设置随机量
        if ((timer <= 0.0) || (!lastAutoMode) || (lastMotionType != motionType)) {
            switch (motionType) {
            case 0: {
                // 全随机模式
                maxTime   = 6.0;
                minTime   = 1.0;
                maxSpeed  = 600;
                minSpeed  = 300;
                direction = (random % 2) * 2 - 1;
            } break;

            case 1: {
                // 巡逻模式
                maxTime   = 0.0;
                minTime   = 0.0;
                maxSpeed  = 600;
                minSpeed  = 600;
                direction = direction;
            } break;

            case 2: {
                // 左右横跳
                maxTime   = 2.0;
                minTime   = 1.0;
                maxSpeed  = 600;
                minSpeed  = 600;
                direction = -direction;
            } break;

            default:
                break;
            }

            timer       = ((float) (random % (int) ((maxTime - minTime) * 10 + 1))) / 10.0 + minTime;
            leftTarget  = direction * (random % (maxSpeed - minSpeed + 1) + minSpeed);
            rightTarget = direction * (random % (maxSpeed - minSpeed + 1) + minSpeed);
        }

        timer -= interval;
        lastAutoMode   = AutoMode;
        lastMotionType = motionType;

        // 关闭自动运动
        if (!AutoMode) {
            leftTarget  = 0;
            rightTarget = 0;
        }

        // 遥控器
        if (ABS(remoteData.lx) > 30) {
            leftTarget  = -1 * remoteData.lx / 660.0f * 600;
            rightTarget = -1 * remoteData.lx / 660.0f * 600;
        }

        // 光电开关
        Left_State  = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
        Right_State = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
        if (Left_State == 0) {
            optoelectronicDirection = -1;
        }
        if (Right_State == 0) {
            optoelectronicDirection = 1;
        }
        if (optoelectronicDirection != 0) {
            direction               = optoelectronicDirection;
            leftTarget              = optoelectronicDirection * ABS(leftTarget);
            rightTarget             = optoelectronicDirection * ABS(rightTarget);
            optoelectronicDirection = 0;
        }

        // 功率限制
        power       = Judge.powerHeatData.chassis_power;                                   // 裁判系统功率
        powerBuffer = Judge.powerHeatData.chassis_power_buffer;                            // 裁判系统功率缓冲
        targetPower = 20.0 - WANG(160.0 - ChassisData.powerBuffer, 0, 160) / 160.0 * 20.0; // 设置目标功率
        Chassis_Limit_Power(&ChassisData, targetPower, power, powerBuffer, interval);      // 根据功率限幅

        // 计算输出电流PID
        PID_Calculate(&PID_Chassis_Left, leftTarget, Motor_Chassis_Left.speed * RPM2RPS);
        PID_Calculate(&PID_Chassis_Right, rightTarget, Motor_Chassis_Right.speed * RPM2RPS);

        // 输出电流值到电调
        Can_Send(CAN1, 0x200, PID_Chassis_Left.output * ChassisData.powerScale, PID_Chassis_Right.output * ChassisData.powerScale, 0, 0);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试数据
        DebugData.debug1 = PID_Stir_Speed.output;
        // DebugData.debug2 = Right_State;
        // DebugData.debug3 = PID_Chassis_Left.feedback;
        // DebugData.debug4 = timer;
    }

    vTaskDelete(NULL);
}

void Task_Gimbal(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 反馈值
    float yawAngle;
    float yawSpeed;
    float pitchAngle;
    float pitchSpeed;
    float pitchAngleLimit;

    // 目标值
    float yawAngleTarget          = 0;
    float pitchAngleTarget        = 0;
    float yawAngleTargetRotate    = 0;
    float pitchAngleTargetRotate  = 0;
    float yawAngleTargetControl   = 0;
    float pitchAngleTargetControl = 0;
    float yawAngleTargetPs        = 0;
    float pitchAngleTargetPs      = 0;

    // 限位值
    int pitchAngleLimitMax     = INT_MAX;
    int pitchAngleLimitMin     = INT_MIN;
    int autoPitchAngleLimitMax = INT_MAX;
    int autoPitchAngleLimitMin = INT_MIN;

    // 视觉系统
    int lastSeq    = 0;
    int counter    = 0;
    int maxTimeout = 500 / intervalms;

    // 自动转头
    int directionX = 1;
    int directionY = 1;

    // 初始化云台PID
    PID_Init(&PID_Stabilizer_Yaw_Angle, 10, 0, 0, 5000, 0);
    PID_Init(&PID_Stabilizer_Yaw_Speed, 250, 0, 0, 12000, 0);
    PID_Init(&PID_Stabilizer_Pitch_Angle, 20, 0.1, 0, 2000, 1000);
    PID_Init(&PID_Stabilizer_Pitch_Speed, 40, 0, 0, 5000, 0);

    while (1) {
        // 设置反馈
        yawAngle         = Motor_Stabilizer_Yaw.angle;
        yawSpeed         = Motor_Stabilizer_Yaw.speed * RPM2RPS;
        pitchAngle       = Motor_Stabilizer_Pitch.angle;
        pitchSpeed       = Motor_Stabilizer_Pitch.speed * RPM2RPS;
        pitchAngleLimit  = -1 * Gyroscope_EulerData.pitch;
        yawAngleTarget   = 0;
        pitchAngleTarget = 0;

        // 设置限位
        if (pitchAngleLimit >= PITCH_ANGLE_MAX && pitchAngleLimitMax == INT_MAX) {
            pitchAngleLimitMax = pitchAngle;
        }
        if (pitchAngleLimit <= PITCH_ANGLE_MIN && pitchAngleLimitMin == INT_MIN) {
            pitchAngleLimitMin = pitchAngle;
        }
        if (pitchAngleLimit >= AUTO_PITCH_ANGLE_MAX && autoPitchAngleLimitMax == INT_MAX) {
            autoPitchAngleLimitMax = pitchAngle;
        }
        if (pitchAngleLimit <= AUTO_PITCH_ANGLE_MIN && autoPitchAngleLimitMin == INT_MIN) {
            autoPitchAngleLimitMin = pitchAngle;
        }

        // 视觉系统
        if (!PsEnabled) {
            // yawAngleTargetControl += yawAngleTargetPs;
            // pitchAngleTargetControl += pitchAngleTargetPs;
            yawAngleTargetPs   = 0;
            pitchAngleTargetPs = 0;
            lastSeq            = Ps.autoaimData.seq;
            counter++;
        } else if (lastSeq != Ps.autoaimData.seq) {
            lastSeq = Ps.autoaimData.seq;
            if (Ps.autoaimData.yaw_angle_diff == 0 && Ps.autoaimData.pitch_angle_diff == 0 && Ps.autoaimData.biu_biu_state == 0) {
                counter++;
            } else {
                counter = 0;
                yawAngleTargetPs += Ps.autoaimData.yaw_angle_diff;
                pitchAngleTargetPs += Ps.autoaimData.pitch_angle_diff;
            }
        } else {
            counter++;
        }
        MIAO(yawAngleTargetPs, YAW_ANGLE_MIN - yawAngleTarget, YAW_ANGLE_MAX - yawAngleTarget);
        MIAO(pitchAngleTargetPs, pitchAngleLimitMin - pitchAngleTarget, pitchAngleLimitMax - pitchAngleTarget);
        yawAngleTarget += yawAngleTargetPs;
        pitchAngleTarget += pitchAngleTargetPs;

        // 自动转头
        if ((counter >= maxTimeout) && (AutoMode)) {
            // 丢失目标,自动旋转
            yawAngleTargetRotate += directionX * 1.2;
            pitchAngleTargetRotate += directionY * 0.5;
        }
        if (counter == INT_MAX) {
            counter = maxTimeout;
        }

        MIAO(yawAngleTargetRotate, YAW_ANGLE_MIN - yawAngleTarget, YAW_ANGLE_MAX - yawAngleTarget);
        MIAO(pitchAngleTargetRotate, pitchAngleLimitMin - pitchAngleTarget, pitchAngleLimitMax - pitchAngleTarget);
        yawAngleTarget += yawAngleTargetRotate;
        pitchAngleTarget += pitchAngleTargetRotate;

        if (yawAngleTarget >= AUTO_YAW_ANGLE_MAX) {
            directionX = -1;
        } else if (yawAngleTarget <= AUTO_YAW_ANGLE_MIN) {
            directionX = 1;
        }
        if (pitchAngleTarget >= autoPitchAngleLimitMax) {
            directionY = -1;
        } else if (pitchAngleTarget <= autoPitchAngleLimitMin) {
            directionY = 1;
        }

        // 设置角度目标
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f * 90 * interval;
        if (ABS(remoteData.ry) > 30) pitchAngleTargetControl += remoteData.ry / 660.0f * 90 * interval;

        MIAO(yawAngleTargetControl, YAW_ANGLE_MIN - yawAngleTarget, YAW_ANGLE_MAX - yawAngleTarget);
        MIAO(pitchAngleTargetControl, pitchAngleLimitMin - pitchAngleTarget, pitchAngleLimitMax - pitchAngleTarget);
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;

        MIAO(yawAngleTarget, YAW_ANGLE_MIN, YAW_ANGLE_MAX);
        MIAO(pitchAngleTarget, pitchAngleLimitMin, pitchAngleLimitMax);

        // 计算PID
        PID_Calculate(&PID_Stabilizer_Yaw_Angle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Stabilizer_Yaw_Speed, PID_Stabilizer_Yaw_Angle.output, yawSpeed);
        PID_Calculate(&PID_Stabilizer_Pitch_Angle, pitchAngleTarget, pitchAngle);
        PID_Calculate(&PID_Stabilizer_Pitch_Speed, PID_Stabilizer_Pitch_Angle.output, pitchSpeed);

        // 输出电流
        Can_Send(CAN1, 0x1ff, PID_Stabilizer_Yaw_Speed.output, PID_Stabilizer_Pitch_Speed.output, PID_Stir_Speed.output, 0);
        // Can_Send(CAN1, 0x1ff, 0, 0, 4000, 0);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = Motor_Stir.speed;
        // DebugData.debug2 = Ps.autoaimData.biu_biu_state;
        // DebugData.debug3 = -1 * Gyroscope_EulerData.pitch;
        // DebugData.debug4 = pitchAngleLimitMin;
        // DebugData.debug5 = pitchAngleLimitMax;
        // DebugData.debug6 = pitchAngleTargetPs;
        // DebugData.debug7 = pitchAngleTargetControl;
        // DebugData.debug8 = pitchAngleLimitMin - pitchAngleTarget;
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
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    //堵转检测
    int stop     = 0;
    int lastStop = 0;
    int counter1 = 0;
    int counter2 = 0;

    // PID 初始化
    PID_Init(&PID_Stir_Speed, 60, 0.1, 0, 6000, 3000); //?? 0.1

    // 摩擦轮是否开启
    Snail_State = 0;

    // 热量限制
    int calmDown = 0; // 1:冷却

    // 射击模式
    int shootMode = 0; // 0:停止 1:发射S

    // 视觉系统
    int lastSeq    = 0;
    int counter    = 0;
    int maxTimeout = 500 / intervalms;

    while (1) {

        // 热量限制
        calmDown = (Judge.powerHeatData.shooter_heat0 > 400) ? 1 : 0;

        // 视觉系统
        if (!PsEnabled) {
            lastSeq = Ps.autoaimData.seq;
            counter++;
        } else if (lastSeq != Ps.autoaimData.seq) {
            lastSeq = Ps.autoaimData.seq;
            if (Ps.autoaimData.biu_biu_state == 0) {
                // if (Ps.autoaimData.yaw_angle_diff == 0 && Ps.autoaimData.pitch_angle_diff == 0 && Ps.autoaimData.biu_biu_state == 0) {
                counter++;
            } else {
                counter   = 0;
                shootMode = 1;
            }
        } else {
            counter++;
        }

        if ((counter >= maxTimeout)) {
            shootMode = 0;
        }
        if (counter == INT_MAX) {
            counter = maxTimeout;
        }

        // if (!PsEnabled) {
        //     lastSeq = Ps.autoaimData.seq;
        // } else if (lastSeq != Ps.autoaimData.seq) {
        //     lastSeq   = Ps.autoaimData.seq;
        //     shootMode = Ps.autoaimData.biu_biu_state;
        // } else {
        //     shootMode = 0;
        // }

        // 射击模式
        shootMode = shootMode | StirEnabled;

        // 摩擦轮是否开启
        if (Snail_State == 0) {
            shootMode = 0;
        };

        if ((calmDown == 0) && (shootMode == 1)) {
            PID_Calculate(&PID_Stir_Speed, 400, Motor_Stir.speed * RPM2RPS);
        } else {
            PID_Calculate(&PID_Stir_Speed, 0, Motor_Stir.speed * RPM2RPS);
        }

        // //堵转检测
        // if (PID_Stir_Speed.output > 3500) {
        //     stop = 1;
        // } else {
        //     stop = 0;
        // }

        // if (stop && counter1 < 40) {
        //     counter1 += 1;
        //     PID_Stir_Speed.output = -2000;
        // } else if (counter1 == 40 && counter2 < 150) {
        //     lastStop                = 0;
        //     PID_Stir_Speed.output_I = 0;
        //     PID_Stir_Speed.output   = 0;
        //     counter2 += 1;
        // }

        // if (counter2 == 150) {
        //     counter1 = 0;
        //     counter2 = 0;
        // }

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        DebugData.debug1 = Motor_Stir.speed;
        DebugData.debug2 = Ps.autoaimData.biu_biu_state;
        DebugData.debug3 = shootMode;
    }

    vTaskDelete(NULL);
}

void Task_Snail(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 占空比
    float dutyCycleStart  = 0.376; // 起始
    float dutyCycleMiddle = 0.446; // 启动
    float dutyCycleEnd    = 0.540; // 加速到你想要的

    // 目标占空比
    float dutyCycleRightSnailTarget = 0.376;
    float dutyCycleLeftSnailTarget  = 0.376;

    // 存储需要的两个过程（初始到启动，启动到你想要的速度）
    float dutyCycleRightSnailProgress1 = 0;
    float dutyCycleLeftSnailProgress1  = 0;
    float dutyCycleRightSnailProgress2 = 0;
    float dutyCycleLeftSnailProgress2  = 0;

    // snail电机状态
    int snailState = 0;
    int lastSnailState;

    enum {
        STEP_SNAIL_IDLE,
        STEP_RIGHT_START_TO_MIDDLE,
        STEP_LEFT_START_TO_MIDDLE,
        STEP_RIGHT_MIDDLE_TO_END,
        STEP_LEFT_MIDDLE_TO_END,
    } Step = STEP_SNAIL_IDLE;

    /*来自dji开源，两个snail不能同时启动*/

    while (1) {

        if (LaserEnabled) {
            LASER_ON;
        } else {
            LASER_OFF;
        }

        lastSnailState = snailState;
        snailState     = FrictEnabled;

        switch (Step) {
        case STEP_SNAIL_IDLE:
            if (snailState == 0) {
                Snail_State                  = 0;
                dutyCycleRightSnailTarget    = 0.376;
                dutyCycleLeftSnailTarget     = 0.376;
                dutyCycleRightSnailProgress1 = 0;
                dutyCycleRightSnailProgress2 = 0;
                dutyCycleLeftSnailProgress1  = 0;
                dutyCycleLeftSnailProgress2  = 0;
            } else if (lastSnailState == 0) {
                Step = STEP_RIGHT_START_TO_MIDDLE;
            }
            break;

        case STEP_RIGHT_START_TO_MIDDLE:
            dutyCycleRightSnailTarget = RAMP(dutyCycleStart, dutyCycleMiddle, dutyCycleRightSnailProgress1);
            dutyCycleRightSnailProgress1 += 0.02f;
            if (dutyCycleRightSnailProgress1 > 1) {
                Step = STEP_LEFT_START_TO_MIDDLE;
                vTaskDelay(100);
            }
            break;

        case STEP_LEFT_START_TO_MIDDLE:
            dutyCycleLeftSnailTarget = RAMP(dutyCycleStart, dutyCycleMiddle, dutyCycleLeftSnailProgress1);
            dutyCycleLeftSnailProgress1 += 0.02f;
            if (dutyCycleLeftSnailProgress1 > 1) {
                Step = STEP_RIGHT_MIDDLE_TO_END;
                vTaskDelay(100);
            }

        case STEP_RIGHT_MIDDLE_TO_END:
            dutyCycleRightSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd, dutyCycleRightSnailProgress2);
            dutyCycleRightSnailProgress2 += 0.01f;
            if (dutyCycleRightSnailProgress2 > 1) {
                Step = STEP_LEFT_MIDDLE_TO_END;
            }
            if (Step != STEP_RIGHT_MIDDLE_TO_END) break;

        case STEP_LEFT_MIDDLE_TO_END:
            dutyCycleLeftSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd, dutyCycleLeftSnailProgress2);
            dutyCycleLeftSnailProgress2 += 0.01f;
            if (dutyCycleLeftSnailProgress2 > 1) {
                Snail_State = 1;
                Step        = STEP_SNAIL_IDLE;
            }
            break;

        default:
            break;
        }

        PWM_Set_Compare(&PWM_Snail1, dutyCycleRightSnailTarget * 1250);
        PWM_Set_Compare(&PWM_Snail2, dutyCycleLeftSnailTarget * 1250);

        vTaskDelayUntil(&LastWakeTime, intervalms);

        // DebugData.debug1 = dutyCycleRightSnailTarget * 1000;
        // DebugData.debug2 = dutyCycleLeftSnailTarget * 1000;
        // DebugData.debug3 = Step;
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
        if (KTV_Play(Music_Sky)) break;
        vTaskDelayUntil(&LastWakeTime, 350);
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

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    xTaskCreate(Task_Snail, "Task_Snail", 500, NULL, 6, NULL);
    xTaskCreate(Task_Stir, "Task_Stir", 400, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}