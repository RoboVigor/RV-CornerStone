/**
 * @brief 哨兵
 * @version 1.2.0
 */
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"

#define IS_DOWN_BOARD Board_Id == 1
#define IS_UP_BOARD Board_Id == 2

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2 && remoteData.switchLeft == 2) {
            vTaskSuspendAll();
            while (1) {
                Motor_Down_Gimbal_Yaw.input   = 0;
                Motor_Down_Gimbal_Pitch.input = 0;
                vTaskDelay(2);
            }
        }
        vTaskDelay(2);
    }
    vTaskDelete(NULL);
}

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        // FrictEnabled = remoteData.switchRight == 2;
        FrictEnabled = 1;
        // LaserEnabled = remoteData.switchLeft == 3;
        // StirEnabled  = (remoteData.switchLeft == 3) && (remoteData.switchRight == 1);
        // StirEnabled = remoteData.switchLeft == 2;
        StirEnabled = 1;
        // PsEnabled   = remoteData.switchLeft == 2;
        // AutoMode    = (remoteData.switchLeft == 2) && (remoteData.switchRight == 1);
        SafetyMode = remoteData.switchRight == 2;

        // if ((remoteData.switchLeft == 1 && remoteData.switchRight == 1) || (!remoteData.state)) {
        //     FrictEnabled = 1;
        //     LaserEnabled = 0;
        //     StirEnabled  = 0;
        //     PsEnabled    = 1;
        //     AutoMode     = 1;
        // }

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
            lastSeq = Node_Host.receiveSeq;
            visionCounter++;
        } else if (lastSeq != Node_Host.receiveSeq) {
            lastSeq = Node_Host.receiveSeq;
            if (ProtocolData.autoaimData.yaw_angle_diff == 0 && ProtocolData.autoaimData.pitch_angle_diff == 0 && ProtocolData.autoaimData.biu_biu_state == 0) {
                visionCounter++;
            } else {
                visionCounter = 0;
                motionType    = 2;
            }
        } else {
            visionCounter++;
        }

        // 挨打
        if (ProtocolData.robotState.remain_HP == lastBlood) {
            hurtCounter++;
        } else {
            hurtCounter = 0;
            motionType  = 2;
        }
        lastBlood = ProtocolData.robotState.remain_HP;

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
        power       = ProtocolData.powerHeatData.chassis_power;                            // 裁判系统功率
        powerBuffer = ProtocolData.powerHeatData.chassis_power_buffer;                     // 裁判系统功率缓冲
        targetPower = 20.0 - WANG(160.0 - ChassisData.powerBuffer, 0, 160) / 160.0 * 20.0; // 设置目标功率
        Chassis_Limit_Power(&ChassisData, targetPower, power, powerBuffer, interval);      // 根据功率限幅

        // 计算输出电流PID
        PID_Calculate(&PID_Chassis_Left, leftTarget, Motor_Chassis_Left.speed * RPM2RPS);
        PID_Calculate(&PID_Chassis_Right, rightTarget, Motor_Chassis_Right.speed * RPM2RPS);

        // 输出电流值到电调
        Motor_Chassis_Left.input  = PID_Chassis_Left.output * ChassisData.powerScale;
        Motor_Chassis_Right.input = PID_Chassis_Right.output * ChassisData.powerScale;

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试数据
        // DebugData.debug1 = PID_Stir_Speed.output;
        // DebugData.debug2 = Right_State;
        // DebugData.debug3 = PID_Chassis_Left.feedback;
        // DebugData.debug4 = timer;
    }

    vTaskDelete(NULL);
}

void Task_Up_Gimbal(void *Parameters) {
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

    // 视觉系统
    int lastSeq    = 0;
    int counter    = 0;
    int maxTimeout = 500 / intervalms;

    // 自动转头
    int directionX = 1;
    int directionY = 1;

    // 初始化云台PID
    PID_Init(&PID_Up_Gimbal_Yaw_Angle, 10, 0, 0, 5000, 0);
    PID_Init(&PID_Up_Gimbal_Yaw_Speed, 250, 0, 0, 12000, 0);
    PID_Init(&PID_Up_Gimbal_Pitch_Angle, 20, 0.1, 0, 2000, 1000);
    PID_Init(&PID_Up_Gimbal_Pitch_Speed, 40, 0, 0, 5000, 0);

    while (1) {
        // 设置反馈
        yawAngle         = Motor_Up_Gimbal_Yaw.angle;
        yawSpeed         = Motor_Up_Gimbal_Yaw.speed * RPM2RPS;
        pitchAngle       = Motor_Up_Gimbal_Pitch.angle;
        pitchSpeed       = Motor_Up_Gimbal_Pitch.speed * RPM2RPS;
        yawAngleTarget   = 0;
        pitchAngleTarget = 0;

        // 视觉系统
        // if (!PsEnabled) {
        //     // yawAngleTargetControl += yawAngleTargetPs;
        //     // pitchAngleTargetControl += pitchAngleTargetPs;
        //     yawAngleTargetPs   = 0;
        //     pitchAngleTargetPs = 0;
        //     lastSeq            = Node_Host.receiveSeq;
        //     counter++;
        // } else if (lastSeq != Node_Host.receiveSeq) {
        //     lastSeq = Node_Host.receiveSeq;
        //     if (ProtocolData.autoaimData.yaw_angle_diff == 0 && ProtocolData.autoaimData.pitch_angle_diff == 0 &&
        //         ProtocolData.autoaimData.biu_biu_state == 0) {
        //         counter++;
        //     } else {
        //         counter = 0;
        //         yawAngleTargetPs += ProtocolData.autoaimData.yaw_angle_diff;
        //         pitchAngleTargetPs += ProtocolData.autoaimData.pitch_angle_diff;
        //     }
        // } else {
        //     counter++;
        // }
        // MIAO(yawAngleTargetPs, YAW_ANGLE_MIN - yawAngleTarget, YAW_ANGLE_MAX - yawAngleTarget);
        // MIAO(pitchAngleTargetPs, pitchAngleLimitMin - pitchAngleTarget, pitchAngleLimitMax - pitchAngleTarget);
        // yawAngleTarget += yawAngleTargetPs;
        // pitchAngleTarget += pitchAngleTargetPs;

        // // 自动转头
        // if ((counter >= maxTimeout) && (AutoMode)) {
        //     // 丢失目标,自动旋转
        //     yawAngleTargetRotate += directionX * 1.2;
        //     pitchAngleTargetRotate += directionY * 0.5;
        // }
        // if (counter == INT_MAX) {
        //     counter = maxTimeout;
        // }

        // MIAO(yawAngleTargetRotate, YAW_ANGLE_MIN - yawAngleTarget, YAW_ANGLE_MAX - yawAngleTarget);
        // MIAO(pitchAngleTargetRotate, pitchAngleLimitMin - pitchAngleTarget, pitchAngleLimitMax - pitchAngleTarget);
        // yawAngleTarget += yawAngleTargetRotate;
        // pitchAngleTarget += pitchAngleTargetRotate;

        // if (yawAngleTarget >= AUTO_YAW_ANGLE_MAX) {
        //     directionX = -1;
        // } else if (yawAngleTarget <= AUTO_YAW_ANGLE_MIN) {
        //     directionX = 1;
        // }
        // if (pitchAngleTarget >= autoPitchAngleLimitMax) {
        //     directionY = -1;
        // } else if (pitchAngleTarget <= autoPitchAngleLimitMin) {
        //     directionY = 1;
        // }

        // 设置角度目标
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f * 90 * interval;
        if (ABS(remoteData.ry) > 30) pitchAngleTargetControl += remoteData.ry / 660.0f * 90 * interval;

        MIAO(yawAngleTargetControl, UP_YAW_ANGLE_MIN - yawAngleTarget, UP_YAW_ANGLE_MAX - yawAngleTarget);
        MIAO(pitchAngleTargetControl, UP_PITCH_ANGLE_MIN - pitchAngleTarget, UP_PITCH_ANGLE_MAX - pitchAngleTarget);
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;

        MIAO(yawAngleTarget, UP_YAW_ANGLE_MIN, UP_YAW_ANGLE_MAX);
        MIAO(pitchAngleTarget, UP_PITCH_ANGLE_MIN, UP_PITCH_ANGLE_MAX);

        MIAO(yawAngleTargetControl, UP_YAW_ANGLE_MIN - yawAngleTarget, UP_YAW_ANGLE_MAX - yawAngleTarget);
        MIAO(pitchAngleTargetControl, UP_PITCH_ANGLE_MIN - pitchAngleTarget, UP_PITCH_ANGLE_MAX - pitchAngleTarget);
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;

        MIAO(yawAngleTarget, UP_YAW_ANGLE_MIN, UP_YAW_ANGLE_MAX);
        MIAO(pitchAngleTarget, UP_PITCH_ANGLE_MIN, UP_PITCH_ANGLE_MAX);

        // 计算PID
        PID_Calculate(&PID_Up_Gimbal_Yaw_Angle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Up_Gimbal_Yaw_Speed, PID_Up_Gimbal_Yaw_Angle.output, yawSpeed);
        PID_Calculate(&PID_Up_Gimbal_Pitch_Angle, pitchAngleTarget, pitchAngle);
        PID_Calculate(&PID_Up_Gimbal_Pitch_Speed, PID_Up_Gimbal_Pitch_Angle.output, pitchSpeed);

        // 输出电流
        Motor_Up_Gimbal_Yaw.input   = PID_Up_Gimbal_Yaw_Speed.output;
        Motor_Up_Gimbal_Pitch.input = PID_Up_Gimbal_Pitch_Speed.output;

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = Motor_Stir.speed;
        // DebugData.debug2 = ProtocolData.autoaimData.biu_biu_state;
        // DebugData.debug3 = -1 * Gyroscope_EulerData.pitch;
        // DebugData.debug4 = pitchAngleLimitMin;
        // DebugData.debug5 = pitchAngleLimitMax;
        // DebugData.debug6 = pitchAngleTargetPs;
        // DebugData.debug7 = pitchAngleTargetControl;
        // DebugData.debug8 = pitchAngleLimitMin - pitchAngleTarget;
    }
    vTaskDelete(NULL);
}

void Task_Down_Gimbal(void *Parameters) {
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

    // 视觉系统
    int lastSeq    = 0;
    int counter    = 0;
    int maxTimeout = 500 / intervalms;

    // 自动转头
    int directionX = 1;
    int directionY = 1;

    // 初始化云台PID
    PID_Init(&PID_Down_Gimbal_Yaw_Angle, 0.2, 0, 0, 500, 1000);
    PID_Init(&PID_Down_Gimbal_Yaw_Speed, 0.4, 0, 0, 2000, 1000);
    PID_Init(&PID_Down_Gimbal_Pitch_Angle, 4.9, 0.05, 7.5, 4000, 1000);
    PID_Init(&PID_Down_Gimbal_Pitch_Speed, 6.9, 0.05, 7.5, 8000, 500);

    while (1) {
        // 设置反馈
        yawAngle         = Motor_Down_Gimbal_Yaw.angle;
        yawSpeed         = Motor_Down_Gimbal_Yaw.speed * RPM2RPS;
        pitchAngle       = Motor_Down_Gimbal_Pitch.angle;
        pitchSpeed       = Motor_Down_Gimbal_Pitch.speed * RPM2RPS;
        yawAngleTarget   = 0;
        pitchAngleTarget = 0;

        // 视觉系统
        // if (!PsEnabled) {
        //     // yawAngleTargetControl += yawAngleTargetPs;
        //     // pitchAngleTargetControl += pitchAngleTargetPs;
        //     yawAngleTargetPs   = 0;
        //     pitchAngleTargetPs = 0;
        //     lastSeq            = Node_Host.receiveSeq;
        //     counter++;
        // } else if (lastSeq != Node_Host.receiveSeq) {
        //     lastSeq = Node_Host.receiveSeq;
        //     if (ProtocolData.autoaimData.yaw_angle_diff == 0 && ProtocolData.autoaimData.pitch_angle_diff == 0 &&
        //         ProtocolData.autoaimData.biu_biu_state == 0) {
        //         counter++;
        //     } else {
        //         counter = 0;
        //         yawAngleTargetPs += ProtocolData.autoaimData.yaw_angle_diff;
        //         pitchAngleTargetPs += ProtocolData.autoaimData.pitch_angle_diff;
        //     }
        // } else {
        //     counter++;
        // }
        // MIAO(yawAngleTargetPs, YAW_ANGLE_MIN - yawAngleTarget, YAW_ANGLE_MAX - yawAngleTarget);
        // MIAO(pitchAngleTargetPs, pitchAngleLimitMin - pitchAngleTarget, pitchAngleLimitMax - pitchAngleTarget);
        // yawAngleTarget += yawAngleTargetPs;
        // pitchAngleTarget += pitchAngleTargetPs;

        // // 自动转头
        // if ((counter >= maxTimeout) && (AutoMode)) {
        //     // 丢失目标,自动旋转
        //     yawAngleTargetRotate += directionX * 1.2;
        //     pitchAngleTargetRotate += directionY * 0.5;
        // }
        // if (counter == INT_MAX) {
        //     counter = maxTimeout;
        // }

        // MIAO(yawAngleTargetRotate, YAW_ANGLE_MIN - yawAngleTarget, YAW_ANGLE_MAX - yawAngleTarget);
        // MIAO(pitchAngleTargetRotate, pitchAngleLimitMin - pitchAngleTarget, pitchAngleLimitMax - pitchAngleTarget);
        // yawAngleTarget += yawAngleTargetRotate;
        // pitchAngleTarget += pitchAngleTargetRotate;

        // if (yawAngleTarget >= AUTO_YAW_ANGLE_MAX) {
        //     directionX = -1;
        // } else if (yawAngleTarget <= AUTO_YAW_ANGLE_MIN) {
        //     directionX = 1;
        // }
        // if (pitchAngleTarget >= autoPitchAngleLimitMax) {
        //     directionY = -1;
        // } else if (pitchAngleTarget <= autoPitchAngleLimitMin) {
        //     directionY = 1;
        // }

        // 设置角度目标
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f * 30000 * interval;
        if (ABS(remoteData.ry) > 30) pitchAngleTargetControl += remoteData.ry / 660.0f * 300 * interval;

        // MIAO(yawAngleTargetControl, DOWN_YAW_ANGLE_MIN - yawAngleTarget, DOWN_YAW_ANGLE_MAX - yawAngleTarget);
        // MIAO(pitchAngleTargetControl, DOWN_PITCH_ANGLE_MIN - pitchAngleTarget, DOWN_PITCH_ANGLE_MAX - pitchAngleTarget);
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;

        // MIAO(yawAngleTarget, DOWN_YAW_ANGLE_MIN, DOWN_YAW_ANGLE_MAX);
        // MIAO(pitchAngleTarget, DOWN_PITCH_ANGLE_MIN, DOWN_PITCH_ANGLE_MAX);

        // MIAO(yawAngleTargetControl, DOWN_YAW_ANGLE_MIN - yawAngleTarget, DOWN_YAW_ANGLE_MAX - yawAngleTarget);
        // MIAO(pitchAngleTargetControl, DOWN_PITCH_ANGLE_MIN - pitchAngleTarget, DOWN_PITCH_ANGLE_MAX - pitchAngleTarget);
        // yawAngleTarget += yawAngleTargetControl;
        // pitchAngleTarget += pitchAngleTargetControl;

        // MIAO(yawAngleTarget, DOWN_YAW_ANGLE_MIN, DOWN_YAW_ANGLE_MAX);
        // MIAO(pitchAngleTarget, DOWN_PITCH_ANGLE_MIN, DOWN_PITCH_ANGLE_MAX);

        // 计算PID
        PID_Calculate(&PID_Down_Gimbal_Yaw_Angle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Down_Gimbal_Yaw_Speed, PID_Down_Gimbal_Yaw_Angle.output, yawSpeed);
        PID_Calculate(&PID_Down_Gimbal_Pitch_Angle, pitchAngleTarget, pitchAngle);
        PID_Calculate(&PID_Down_Gimbal_Pitch_Speed, PID_Down_Gimbal_Pitch_Angle.output, pitchSpeed);

        // 输出电流
        Motor_Down_Gimbal_Yaw.input   = 40 * PID_Down_Gimbal_Yaw_Speed.output;
        Motor_Down_Gimbal_Pitch.input = 40 * PID_Down_Gimbal_Pitch_Speed.output;

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        DebugData.debug1 = pitchAngleTargetControl;
        DebugData.debug2 = Motor_Down_Gimbal_Pitch.angle * 100;
        DebugData.debug3 = pitchAngleTarget;
        DebugData.debug4 = pitchSpeed;
        DebugData.debug5 = Motor_Down_Gimbal_Pitch.input;
        DebugData.debug6 = PID_Down_Gimbal_Pitch_Speed.output;
        DebugData.debug7 = remoteData.ry;
        DebugData.debug8 = PID_Down_Gimbal_Pitch_Angle.output;
        // DebugData.debug8 = pitchAngleLimitMin - pitchAngleTarget;
    }
    vTaskDelete(NULL);
}

void Task_Up_Stir(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    //堵转检测
    int stop     = 0;
    int lastStop = 0;
    int counter1 = 0;
    int counter2 = 0;

    // PID 初始化
    PID_Init(&PID_Up_Stir_Speed, 12, 0, 0, 6000, 3000);

    // 热量限制
    int calmDown = 0; // 1:冷却

    // 射击模式
    int shootMode = 1; // 0:停止 1:发射S

    // 视觉系统
    int lastSeq    = 0;
    int counter    = 0;
    int maxTimeout = 500 / intervalms;

    int targetSpeed = 0;

    while (1) {
        targetSpeed = CHOOSEL(0, 200, 300);

        PID_Calculate(&PID_Up_Stir_Speed, targetSpeed, Motor_Up_Stir.speed * RPM2RPS);

        if (StirEnabled) {
            Motor_Up_Stir.input = PID_Up_Stir_Speed.output;
        }

        // // 热量限制
        // calmDown = (ProtocolData.powerHeatData.shooter_heat0 > 400) ? 1 : 0;

        // // 视觉系统
        // if (!PsEnabled) {
        //     lastSeq = Node_Host.receiveSeq;
        //     counter++;
        // } else if (lastSeq != Node_Host.receiveSeq) {
        //     lastSeq = Node_Host.receiveSeq;
        //     if (ProtocolData.autoaimData.biu_biu_state == 0) {
        //         // if (ProtocolData.autoaimData.yaw_angle_diff == 0 && ProtocolData.autoaimData.pitch_angle_diff == 0 &&
        //         // ProtocolData.autoaimData.biu_biu_state == 0) {
        //         counter++;
        //     } else {
        //         counter   = 0;
        //         shootMode = 1;
        //     }
        // } else {
        //     counter++;
        // }

        // if ((counter >= maxTimeout)) {
        //     shootMode = 0;
        // }
        // if (counter == INT_MAX) {
        //     counter = maxTimeout;
        // }

        // if (!PsEnabled) {
        //     lastSeq = Node_Host.receiveSeq;
        // } else if (lastSeq != Node_Host.receiveSeq) {
        //     lastSeq   = Node_Host.receiveSeq;
        //     shootMode = ProtocolData.autoaimData.biu_biu_state;
        // } else {
        //     shootMode = 0;
        // }

        // 射击模式
        // shootMode = shootMode | StirEnabled;

        // // 摩擦轮是否开启
        // if (Snail_State == 0) {
        //     shootMode = 0;
        // };

        // if ((calmDown == 0) && (shootMode == 1)) {
        //     PID_Calculate(&PID_Stir_Speed, 400, Motor_Stir.speed * RPM2RPS);
        // } else {
        //     PID_Calculate(&PID_Stir_Speed, 0, Motor_Stir.speed * RPM2RPS);
        // }

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

        // DebugData.debug2 = ProtocolData.autoaimData.biu_biu_state;
        // DebugData.debug3 = shootMode;
    }

    vTaskDelete(NULL);
}

void Task_Down_Stir(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    //堵转检测
    int stop     = 0;
    int lastStop = 0;
    int counter1 = 0;
    int counter2 = 0;

    // PID 初始化
    PID_Init(&PID_Down_Stir_Speed, 12, 0, 0, 6000, 3000);

    // 热量限制
    int calmDown = 0; // 1:冷却

    // 射击模式
    int shootMode = 1; // 0:停止 1:发射S

    // 视觉系统
    int lastSeq    = 0;
    int counter    = 0;
    int maxTimeout = 500 / intervalms;

    int targetSpeed = 0;

    while (1) {
        targetSpeed = CHOOSEL(0, 200, 300);

        PID_Calculate(&PID_Down_Stir_Speed, targetSpeed, Motor_Down_Stir.speed * RPM2RPS);

        if (StirEnabled) {
            Motor_Down_Stir.input = PID_Down_Stir_Speed.output;
        }

        // // 热量限制
        // calmDown = (ProtocolData.powerHeatData.shooter_heat0 > 400) ? 1 : 0;

        // // 视觉系统
        // if (!PsEnabled) {
        //     lastSeq = Node_Host.receiveSeq;
        //     counter++;
        // } else if (lastSeq != Node_Host.receiveSeq) {
        //     lastSeq = Node_Host.receiveSeq;
        //     if (ProtocolData.autoaimData.biu_biu_state == 0) {
        //         // if (ProtocolData.autoaimData.yaw_angle_diff == 0 && ProtocolData.autoaimData.pitch_angle_diff == 0 &&
        //         // ProtocolData.autoaimData.biu_biu_state == 0) {
        //         counter++;
        //     } else {
        //         counter   = 0;
        //         shootMode = 1;
        //     }
        // } else {
        //     counter++;
        // }

        // if ((counter >= maxTimeout)) {
        //     shootMode = 0;
        // }
        // if (counter == INT_MAX) {
        //     counter = maxTimeout;
        // }

        // if (!PsEnabled) {
        //     lastSeq = Node_Host.receiveSeq;
        // } else if (lastSeq != Node_Host.receiveSeq) {
        //     lastSeq   = Node_Host.receiveSeq;
        //     shootMode = ProtocolData.autoaimData.biu_biu_state;
        // } else {
        //     shootMode = 0;
        // }

        // 射击模式
        // shootMode = shootMode | StirEnabled;

        // // 摩擦轮是否开启
        // if (Snail_State == 0) {
        //     shootMode = 0;
        // };

        // if ((calmDown == 0) && (shootMode == 1)) {
        //     PID_Calculate(&PID_Stir_Speed, 400, Motor_Stir.speed * RPM2RPS);
        // } else {
        //     PID_Calculate(&PID_Stir_Speed, 0, Motor_Stir.speed * RPM2RPS);
        // }

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

        // DebugData.debug2 = ProtocolData.autoaimData.biu_biu_state;
        // DebugData.debug3 = shootMode;
    }

    vTaskDelete(NULL);
}

void Task_Up_Frict(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    float motorUpLeftSpeed;
    float motorUpRightSpeed;
    float targetSpeed = 0;

    // 上
    PID_Init(&PID_Up_Frict_Left_Speed, 50, 0, 0, 16384, 2000);
    PID_Init(&PID_Up_Frict_Right_Speed, 50, 0, 0, 16384, 2000);

    while (1) {
        targetSpeed = CHOOSER(0, 200, 300);

        motorUpLeftSpeed  = Motor_Up_Frict_Left.speed / 19.2f;
        motorUpRightSpeed = Motor_Up_Frict_Right.speed / 19.2f;

        PID_Calculate(&PID_Up_Frict_Left_Speed, -1 * targetSpeed, motorUpLeftSpeed);
        PID_Calculate(&PID_Up_Frict_Right_Speed, targetSpeed, motorUpRightSpeed);

        // targetSpeed = 200;   //10m/s
        // targetSpeed = 220;   //12m/s
        // targetSpeed = 260;   //15m/s

        if (FrictEnabled) {
            Motor_Up_Frict_Left.input  = PID_Up_Frict_Left_Speed.output;
            Motor_Up_Frict_Right.input = PID_Up_Frict_Right_Speed.output;
        };

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Down_Frict(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    float motorDownLeftSpeed;
    float motorDownRightSpeed;
    float targetSpeed = 0;

    // 下
    PID_Init(&PID_Down_Frict_Left_Speed, 50, 0, 0, 16384, 2000);
    PID_Init(&PID_Down_Frict_Right_Speed, 50, 0, 0, 16384, 2000);

    while (1) {
        targetSpeed = CHOOSER(0, 200, 300);

        motorDownLeftSpeed  = Motor_Down_Frict_Left.speed / 19.2f;
        motorDownRightSpeed = Motor_Down_Frict_Right.speed / 19.2f;

        PID_Calculate(&PID_Down_Frict_Left_Speed, -1 * targetSpeed, motorDownLeftSpeed);
        PID_Calculate(&PID_Down_Frict_Right_Speed, targetSpeed, motorDownRightSpeed);

        // targetSpeed = 200;   //10m/s
        // targetSpeed = 220;   //12m/s
        // targetSpeed = 260;   //15m/s

        if (FrictEnabled) {
            Motor_Down_Frict_Left.input  = PID_Down_Frict_Left_Speed.output;
            Motor_Down_Frict_Right.input = PID_Down_Frict_Right_Speed.output;
        };

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Can_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        Bridge_Send_Motor(&BridgeData, SafetyMode);
        vTaskDelayUntil(&LastWakeTime, intervalms); // 发送频率
    }
    vTaskDelete(NULL);
}

void Task_Board_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    uint16_t id;
    uint16_t dataLength;

    while (1) {
        // if (IS_DOWN_BOARD) {
        //     id                                        = 0x501;
        //     ProtocolData.user.boardGimbalUp.remoteRx  = remoteData.rx;
        //     ProtocolData.user.boardGimbalUp.remoteRy  = remoteData.ry;
        //     ProtocolData.user.boardGimbalUp.remoteSwL = remoteData.switchLeft;
        //     ProtocolData.user.boardGimbalUp.remoteSwR = remoteData.switchRight;
        // } else if (IS_UP_BOARD) {
        //     remoteData.rx          = ProtocolData.user.boardGimbalUp.remoteRx;
        //     remoteData.ry          = ProtocolData.user.boardGimbalUp.remoteRy;
        //     remoteData.switchLeft  = ProtocolData.user.boardGimbalUp.remoteSwL;
        //     remoteData.switchRight = ProtocolData.user.boardGimbalUp.remoteSwR;
        //     Bridge_Send_Protocol_Once(&Node_Board, 0x501);
        // }

        // // Can发送
        // dataLength = Protocol_Pack(&UserChannel, id);
        // Can_Send_Msg(CAN1, id, UserChannel.sendBuf, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);

        // // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
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
        if (KTV_Play(Music_Bird)) break;
        vTaskDelayUntil(&LastWakeTime, 350);
    }
    vTaskDelete(NULL);
}
