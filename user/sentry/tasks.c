/**
 * @brief 哨兵
 * @version 1.2.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            Can_Send(CAN1, 0x1ff, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(2);
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

    // 随机模式
    uint16_t random = 0;
    float    timer  = 0.0;
    int      lastSwitch;
    float    maxTime  = 2;   // 3
    float    minTime  = 1;   // 1.5
    int      maxSpeed = 600; // 480
    int      minSpeed = 360; // 240

    // 光电开关
    int direction;

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
        // 随机模式
        srand(Motor_Chassis_Left.position);
        random = rand();
        if ((timer <= 0.0) || (lastSwitch == 3)) {
            lastSwitch  = remoteData.switchLeft;
            direction   = 0;
            timer       = ((float) (random % (int) ((maxTime - minTime) * 10 + 1))) / 10.0 + minTime;
            leftTarget  = ((random % 2) * 2 - 1) * (random % (maxSpeed - minSpeed + 1) + minSpeed);
            rightTarget = ((random % 2) * 2 - 1) * (random % (maxSpeed - minSpeed + 1) + minSpeed);
        }
        if (remoteData.switchLeft == 3) {
            lastSwitch  = 3;
            leftTarget  = 0;
            rightTarget = 0;
        }
        timer -= interval;

        // 遥控器
        if ((ABS(remoteData.lx) > 30) && (ABS(remoteData.ly) < 30)) {
            leftTarget  = -1 * remoteData.lx / 660.0f * 600;
            rightTarget = -1 * remoteData.lx / 660.0f * 600;
        }

        // 光电开关
        Left_State  = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);
        Right_State = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
        if (Left_State == 0) {
            direction = -1;
        }
        if (Right_State == 0) {
            direction = 1;
        }
        if (direction != 0) {
            leftTarget  = direction * ABS(leftTarget);
            rightTarget = direction * ABS(rightTarget);
        }

        // 功率限制
        power       = Judge.powerHeatData.chassis_power;                                  // 裁判系统功率
        powerBuffer = Judge.powerHeatData.chassis_power_buffer;                           // 裁判系统功率缓冲                      // 裁判系统功率
        targetPower = 20.0 - WANG(200.0 - ChassisData.powerBuffer, 0, 40) / 200.0 * 20.0; // 设置目标功率
        Chassis_Limit_Power(&ChassisData, targetPower, power, powerBuffer, interval);     // 根据功率限幅

        // 计算输出电流PID
        PID_Calculate(&PID_Chassis_Left, leftTarget, Motor_Chassis_Left.speed * RPM2RPS);
        PID_Calculate(&PID_Chassis_Right, rightTarget, Motor_Chassis_Right.speed * RPM2RPS);

        // 输出电流值到电调
        Can_Send(CAN1, 0x200, PID_Chassis_Left.output, PID_Chassis_Right.output, 0, 0);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试数据
        // DebugData.debug1 = Motor_Chassis_Left.speed;
        // DebugData.debug2 = leftTarget;
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

    // 目标值
    float yawAngleTarget   = 0;
    float pitchAngleTarget = 0;

    // 视觉系统
    int   lastSeq            = 0;
    float psYawAngleTarget   = 0;
    float psPitchAngleTarget = 0;

    //自动转头
    int counter   = 0;
    int direction = 1;

    // 初始化云台PID
    PID_Init(&PID_Stabilizer_Yaw_Angle, 30, 0, 0, 16000, 0);
    PID_Init(&PID_Stabilizer_Yaw_Speed, 60, 0, 0, 5000, 0);
    PID_Init(&PID_Stabilizer_Pitch_Angle, 20, 0, 0, 16000, 0);
    PID_Init(&PID_Stabilizer_Pitch_Speed, 20, 0.1, 0, 4000, 1000);

    while (1) {
        // 设置反馈
        yawAngle   = Motor_Stabilizer_Yaw.angle;
        yawSpeed   = Motor_Stabilizer_Yaw.speed * RPM2RPS;
        pitchAngle = -1 * Gyroscope_EulerData.pitch; // 逆时针为正
        pitchSpeed = ImuData.gx / GYROSCOPE_LSB;     // 逆时针为正

        //视觉系统
        if (remoteData.switchLeft == 3) {
            lastSeq = Ps.autoaimData.seq;
        } else if (lastSeq != Ps.autoaimData.seq) {
            lastSeq = Ps.autoaimData.seq;
            counter = 0;
            psYawAngleTarget += Ps.autoaimData.yaw_angle_diff;
            psPitchAngleTarget -= Ps.autoaimData.pitch_angle_diff;
        } else {
            counter++;
        }

        // 自动转头
        if (yawAngleTarget == 120) {
            direction = -1;
        } else if (yawAngleTarget == -120) {
            direction = 1;
        }
        if ((counter >= 10) && (remoteData.switchLeft != 3)) {
            yawAngleTarget += direction * 1;
            pitchAngleTarget = -40;
        }
        if (counter == INT_MAX) {
            counter = 10;
        }

        // 设置角度目标
        if (ABS(remoteData.rx) > 30) yawAngleTarget += remoteData.rx / 660.0f * 0.5;
        if (ABS(remoteData.ry) > 30) pitchAngleTarget += remoteData.ry / 660.0f * 0.5;
        yawAngleTarget += psYawAngleTarget;
        pitchAngleTarget += psPitchAngleTarget;
        MIAO(yawAngleTarget, -120, 120);
        MIAO(pitchAngleTarget, -40, 15);

        // 计算PID
        PID_Calculate(&PID_Stabilizer_Yaw_Angle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Stabilizer_Yaw_Speed, PID_Stabilizer_Yaw_Angle.output, yawSpeed);

        PID_Calculate(&PID_Stabilizer_Pitch_Angle, pitchAngleTarget, pitchAngle);
        PID_Calculate(&PID_Stabilizer_Pitch_Speed, PID_Stabilizer_Pitch_Angle.output, pitchSpeed);

        // 输出电流
        Can_Send(CAN1, 0x1ff, PID_Stabilizer_Yaw_Speed.output, PID_Stabilizer_Pitch_Speed.output, PID_Stir_Speed.output, 0);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = counter;
        // DebugData.debug2 = yawAngleTarget;
        // DebugData.debug3 = lastSeq;
        // DebugData.debug4 = Ps.autoaimData.seq;
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

    // PID 初始化
    PID_Init(&PID_Stir_Speed, 1.2, 0.1, 0, 8000, 3500);

    // 热量限制
    int calmDown = 0; // 1:冷却

    // 射击模式
    int shootMode = 0; // 0:停止 1:发射

    // 视觉系统
    int lastSeq = 0;

    while (1) {

        // 热量限制
        calmDown = ((Judge.powerHeatData.shooter_heat0 >= 420) || ((calmDown = 1) && (Judge.powerHeatData.shooter_heat0 >= 100))) ? 1 : 0;

        // 射击模式
        shootMode = (ABS(remoteData.ly) > 30) ? 1 : 0;

        // 视觉系统
        if (remoteData.switchLeft == 3) {
            lastSeq = Ps.autoaimData.seq;
        } else if (lastSeq != Ps.autoaimData.seq) {
            lastSeq   = Ps.autoaimData.seq;
            shootMode = 1;
        } else {
            shootMode = 0;
        }

        if ((calmDown == 0) && (shootMode == 1)) {
            PID_Calculate(&PID_Stir_Speed, 3500.0, Motor_Stir.speed);
        } else {
            PID_Calculate(&PID_Stir_Speed, 0, Motor_Stir.speed);
        }

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
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
    float dutyCycleEnd    = 0.560; // 加速到你想要的

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

        // GPIO_SetBits(GPIOG, GPIO_Pin_13); //激光

        lastSnailState = snailState;
        snailState     = (remoteData.switchRight != 3) ? 1 : 0;

        switch (Step) {
        case STEP_SNAIL_IDLE:
            if (snailState == 0) {

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
            dutyCycleRightSnailProgress1 += 0.1f;
            if (dutyCycleRightSnailProgress1 > 1) {
                Step = STEP_LEFT_START_TO_MIDDLE;
                vTaskDelay(100);
            }
            break;

        case STEP_LEFT_START_TO_MIDDLE:
            dutyCycleLeftSnailTarget = RAMP(dutyCycleStart, dutyCycleMiddle, dutyCycleLeftSnailProgress1);
            dutyCycleLeftSnailProgress1 += 0.1f;
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
                Step = STEP_SNAIL_IDLE;
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

    // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    xTaskCreate(Task_Snail, "Task_Snail", 500, NULL, 6, NULL);
    xTaskCreate(Task_Stir, "Task_Stir", 400, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}