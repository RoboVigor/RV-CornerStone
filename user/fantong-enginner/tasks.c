/**
 * @brief 工程
 * @version 1.2.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (SafetyMode) {
            vTaskSuspendAll();
            while (1) {
                Can_Send(CAN1, 0x200, 0, 0, 0, 0);
                Can_Send(CAN1, 0x1ff, 0, 0, 0, 0);
                Can_Send(CAN2, 0x2ff, 0, 0, 0, 0);
                GO_OFF;
                GET_OFF;
                RESCUE_HOOK_UP;
                vTaskDelay(2);
            }
        }
        vTaskDelay(2);
    }
    vTaskDelete(NULL);
}

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      interval     = 0.01;            // 任务运行间隔 s
    int        intervalms   = interval * 1000; // 任务运行间隔 ms

    RescueMode = 0;
    while (1) {
        // FetchMode   = LEFT_SWITCH_TOP;
        ChassisMode = LEFT_SWITCH_TOP;
        RaiseMode   = (LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP);
        // RaiseMode   = LEFT_SWITCH_TOP;
        // FrictEnabled = (LEFT_SWITCH_BOTTOM && (RIGHT_SWITCH_TOP || RIGHT_SWITCH_MIDDLE));
        // StirEnabled  = (LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_TOP);
        SafetyMode = (RIGHT_SWITCH_BOTTOM && LEFT_SWITCH_BOTTOM);
        // if (LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_TOP) {
        //     RescueMode = 1;
        // } else if (LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_BOTTOM) {
        //     RescueMode = 0;
        // }

        // 调试视觉用
        // PsAimEnabled   = (LEFT_SWITCH_TOP) && (remoteData.switchRight != 3);
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 底盘运动
    float vx             = 0;
    float vy             = 0;
    float vw             = 0;
    float vwRamp         = 0;
    float vwRampProgress = 0;

    // 反馈值
    float motorAngle, motorSpeed;
    float lastMotorAngle = Motor_Yaw.angle;

    // 底盘跟随PID
    float followDeadRegion = 3.0;
    PID_Init(&PID_Follow_Angle, 1, 0, 0, 1000, 100);
    PID_Init(&PID_Follow_Speed, 10, 0, 0, 1000, 0);

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, 32, 0, 0, 12000, 4000);
    PID_Init(&PID_LBCM, 32, 0, 0, 12000, 4000);
    PID_Init(&PID_RBCM, 32, 0, 0, 12000, 4000);
    PID_Init(&PID_RFCM, 32, 0, 0, 12000, 4000);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    while (1) {

        // 设置反馈值
        motorAngle = Motor_Yaw.angle;           // 电机角度
        motorSpeed = Motor_Yaw.speed * RPM2RPS; // 电机角速度

        // 底盘跟随死区
        followDeadRegion = 3; // 开启底盘跟随死区
        PID_Calculate(&PID_Follow_Angle, 0, motorAngle);
        PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output, motorSpeed);

        // 根据控制模式设定运动
        if (ChassisMode) {
            vx = -remoteData.lx / 660.0f * 8.0;
            vy = remoteData.ly / 660.0f * 12.0;
        } else {
            vx = 0;
            vy = 0;
        }

        vw = ABS(PID_Follow_Angle.error) < followDeadRegion ? 0 : (-1 * PID_Follow_Speed.output * DPS2RPS);

        //按条件开启底盘K_I
        if (vx < 0.1 && vy < 0.1 && vw < 0.3) {
            PID_LFCM.i = 1;
            PID_LBCM.i = 1;
            PID_RBCM.i = 1;
            PID_RFCM.i = 1;
        } else {
            PID_LFCM.i = 0;
            PID_LBCM.i = 0;
            PID_RBCM.i = 0;
            PID_RFCM.i = 0;
        }

        // 开机时底盘匀速回正
        vwRamp = RAMP(0, vw, vwRampProgress);
        if (vwRampProgress < 1) {
            vwRampProgress += 0.001f;
        }

        // 设置底盘总体移动速度
        Chassis_Update(&ChassisData, vx, vy, vwRamp);

        // 麦轮解算
        Chassis_Calculate_Rotor_Speed(&ChassisData);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调
        if (ChassisMode) {
            Motor_LF.input = PID_LFCM.output;
            Motor_LB.input = PID_LBCM.output;
            Motor_RB.input = PID_RBCM.output;
            Motor_RF.input = PID_RFCM.output;
        } else {
            Motor_LF.input = 0;
            Motor_LB.input = 0;
            Motor_RB.input = 0;
            Motor_RF.input = 0;
        }

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Gimbal(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int16_t    intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 反馈值
    float yawAngle, yawSpeed, pitchAngle, pitchSpeed, chassisAngle;

    // 目标值
    float yawAngleTarget            = 0; // 目标Yaw
    float pitchAngleTarget          = 0; // 目标Pitch
    float yawAngleTargetControl     = 0; // 遥控器输入
    float pitchAngleTargetControl   = 0; // 遥控器输入
    float pitchAngleTargetFix       = 0; // 上坡补偿
    float pitchAngleTargetFixStable = 0; // 上坡补偿

    // 输出量
    int32_t yawCurrent   = 0;
    int32_t pitchCurrent = 0;

    // Pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = Gyroscope_EulerData.pitch - 90;
    float pitchAngleTargetRamp = 0;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 10, 0, 0, 16000, 10);
    PID_Init(&PID_Cloud_YawSpeed, 100, 0, 0, 16000, 0);
    // PID_Init(&PID_Cloud_PitchAngle, 10, 0, 0, 16000, 0);
    // PID_Init(&PID_Cloud_PitchSpeed, 10, 0, 0, 16000, 0);
    // PID_Init(&PID_Cloud_YawAngle, 20, 0, 0, 16000, 0);
    // PID_Init(&PID_Cloud_YawSpeed, 2, 0, 0, 4000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 40, 0, 0, 16000, 0);
    PID_Init(&PID_Cloud_PitchSpeed, 1, 0, 0, 2000, 0);

    while (1) {
        // 重置目标
        yawAngleTarget   = 0;
        pitchAngleTarget = 0;

        // 设置反馈
        yawAngle     = -1 * Gyroscope_EulerData.yaw;    // 逆时针为正
        yawSpeed     = ImuData.gy / GYROSCOPE_LSB;      // 逆时针为正
        pitchAngle   = Gyroscope_EulerData.pitch - 90;  // 逆时针为正
        pitchSpeed   = -1 * ImuData.gx / GYROSCOPE_LSB; // 逆时针为正
        chassisAngle = Motor_Pitch.angle + pitchAngle;

        // 遥控器输入角度目标
        if (ChassisMode) {
            if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f * 360 * interval;
            if (ABS(remoteData.ry) > 30) pitchAngleTargetControl -= remoteData.ry / 660.0f * 360 * interval;
            yawAngleTargetControl += mouseData.x * 0.5 * 0.005; // 0.005
            pitchAngleTargetControl += mouseData.y * 0.005;
            MIAO(pitchAngleTargetControl, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
            yawAngleTarget += yawAngleTargetControl;
            pitchAngleTarget += pitchAngleTargetControl;
        }

        // 上坡补偿
        // pitchAngleTargetFixStable = FirstOrderLowPassFilter(-1 * (chassisAngle / 40.0) * (GIMBAL_PITCH_MIN - pitchAngleTarget), &pitchAngleTargetFix, 200,
        // 20); pitchAngleTarget += pitchAngleTargetFixStable;

        // 限制云台运动范围
        MIAO(pitchAngleTarget, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

        // 开机时pitch轴匀速抬起
        // pitchAngleTargetRamp = RAMP(pitchRampStart, pitchAngleTarget, pitchRampProgress);
        // if (pitchRampProgress < 1) {
        //     pitchRampProgress += 0.01f;
        // }

        // 计算PID
        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);
        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTarget, pitchAngle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchSpeed);

        // 输出电流
        yawCurrent   = -1 * PID_Cloud_YawSpeed.output;
        pitchCurrent = -1 * PID_Cloud_PitchSpeed.output;
        MIAO(yawCurrent, -12000, 12000);
        // MIAO(pitchCurrent, -12000, 12000);
        MIAO(pitchCurrent, -1500, 1500);
        if (ChassisMode) {
            Motor_Yaw.input   = yawCurrent;
            Motor_Pitch.input = pitchCurrent;
        }
        //任务间隔
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Can_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    CAN_TypeDef *Canx[2]          = {CAN1, CAN2};
    Motor_Type **Canx_Device[2]   = {Can1_Device, Can2_Device};
    uint16_t     Can_Send_Id[3]   = {0x200, 0x1ff, 0x2ff};
    uint16_t     Can_ESC_Id[3][4] = {{0x201, 0x202, 0x203, 0x204}, {0x205, 0x206, 0x207, 0x208}, {0x209, 0x020a, 0x20b, 0x20c}};

    int         i, j, k;     // CAN序号 发送ID序号 电调ID序号
    int         isEmpty = 0; // 同一发送ID下是否有电机
    Motor_Type *motor;       // 根据i,j,k锁定电机
    int16_t     currents[4]; // CAN发送电流

    while (1) {

        for (i = 0; i < 2; i++) {
            for (j = 0; j < 3; j++) {
                isEmpty = 1;
                for (k = 0; k < 4; k++) {
                    motor       = *(Canx_Device[i] + ESC_ID(Can_ESC_Id[j][k]));
                    currents[k] = (motor && motor->inputEnabled) ? motor->input : 0;
                    isEmpty     = isEmpty && (!motor || !(motor->inputEnabled));
                }
                if (!isEmpty) {
                    Can_Send(Canx[i], Can_Send_Id[j], currents[0], currents[1], currents[2], currents[3]);
                }
            }
        }

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Fetch(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms
    TickType_t timer        = xTaskGetTickCount();

    // 斜坡
    float pitchAngleTargetProgress      = 0;
    float pitchAngleTargetProgressDelta = 0.05;
    float pitchAngleTargetStart         = 0;
    float pitchAngleTargetStop          = 10;

    // 反馈值
    float xSpeed;
    float pitchLeftAngle;
    float lastPitchLeftAngle;
    float pitchRightAngle;

    // 目标值
    float xSpeedTarget     = 0;
    float pitchAngleTarget = 0;

    // 视觉辅助
    int   lastSeq        = 0;
    float xSpeedTargetPs = 0;

    // 临时状态
    int16_t timePassed     = 0;
    uint8_t lastFetchState = -1;

    // 初始化PID
    PID_Init(&PID_Fetch_X, 300, 0, 100, 6000, 2000);
    PID_Init(&PID_Fetch_Pitch_Left, 350, 0, 3000, 12000, 1500);
    PID_Init(&PID_Fetch_Pitch_Left, 350, 0, 3000, 12000, 1500);

    // 初始化状态
    FetchState = 0;
    RotateDone = 0;

    while (1) {
        // 设置反馈
        xSpeed          = Motor_Fetch_X.speed * RPM2RPS;
        pitchLeftAngle  = Motor_Fetch_Left_Pitch.angle;
        pitchRightAngle = Motor_Fetch_Right_Pitch.angle;

        // 视觉辅助
        // if (!PsAimEnabled) {
        //     xSpeedTargetPs = 0;
        //     lastSeq        = Ps.autoaimData.seq;
        // } else if (lastSeq != Ps.autoaimData.seq) {
        //     lastSeq        = Ps.autoaimData.seq;
        //     xSpeedTargetPs = Ps.autoaimData.yaw_angle_diff;
        //     if (Ps.autoaimData.pitch_angle_diff > 0) {
        //         pitchAngleTargetProgress = 0;
        //         pitchAngleTargetStart    = pitchAngleTargetStop;
        //         pitchAngleTargetStop     = Ps.autoaimData.pitch_angle_diff;
        //     }
        // }
        // xSpeedTarget = xSpeedTargetPs;

        // 抓取状态更新
        while (1) {
            // xTaskGetTickCount()-timer
            // timer = xTaskGetTickCount();
            if (ChassisMode) {
                FetchState = FetchReset;
            }
            if (FetchState == FetchReset) {
                /* 回到默认位置 */
                if (lastFetchState != FetchReset) {
                    lastFetchState = FetchState;
                    // 爪子归位
                    pitchAngleTargetProgress      = 1;
                    pitchAngleTargetProgressDelta = 1;
                    pitchAngleTargetStart         = -1 * pitchLeftAngle;
                    pitchAngleTargetStop          = 10;
                }
                // 跳转:爪子和平台已归位
                if (RotateDone && FetchMode) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchWaitRaise) {
                /* 等待抬升 */
                if (FantongRaised) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchWaitSignal) {
                /* 平台运动,等待抓取信号 */

                // 遥控器控制左右移动
                if (remoteData.lx > 30) {
                    xSpeedTarget = 200;
                } else if (remoteData.lx < -30) {
                    xSpeedTarget = -200;
                } else {
                    xSpeedTarget = 0;
                }

                // 手动模式:开始抓取
                if (remoteData.switchRight == 2) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchRotateOut) {
                /* 转出去 */
                if (lastFetchState != FetchRotateOut) {
                    lastFetchState = FetchRotateOut;
                    // 转爪子
                    pitchAngleTargetProgress      = 0;
                    pitchAngleTargetProgressDelta = 0.01;
                    pitchAngleTargetStart         = -1 * pitchLeftAngle;
                    pitchAngleTargetStop          = 190;
                    RotateDone                    = 0;
                }
                // 跳转:爪子到位
                if (RotateDone) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchLock) {
                /* 夹住弹药箱 */
                if (GET_STATUS == 0) {
                    GET_ON;
                    // 重置计时器
                    timer = xTaskGetTickCount();
                }
                timePassed = xTaskGetTickCount() - timer;
                // 跳转:计时器
                if (timePassed >= 300) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchRotateIn) {
                /* 转回来 */
                if (lastFetchState != FetchRotateIn) {
                    lastFetchState = FetchRotateIn;
                    // 转爪子
                    pitchAngleTargetProgress      = 0.2;
                    pitchAngleTargetProgressDelta = 0;
                    pitchAngleTargetStart         = -1 * pitchLeftAngle;
                    pitchAngleTargetStop          = 30;
                    RotateDone                    = 0;
                }
                if (pitchLeftAngle < 180) {
                    pitchAngleTargetProgressDelta = 0.002;
                }
                // 跳转:爪子到位
                if (RotateDone) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchEating) {
                /* 喂食 */
                if (lastFetchState != FetchEating) {
                    lastFetchState = FetchEating;
                    // 重置计时器
                    timer = xTaskGetTickCount();
                }
                timePassed = xTaskGetTickCount() - timer;
                // 跳转:喂食完毕
                if (timePassed >= 2000) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchThrow) {
                /* 扔弹药箱 */
                if (lastFetchState != FetchThrow) {
                    lastFetchState = FetchThrow;
                    // 转爪子
                    pitchAngleTargetProgress      = 1;
                    pitchAngleTargetProgressDelta = 0;
                    pitchAngleTargetStart         = -1 * pitchLeftAngle;
                    pitchAngleTargetStop          = 180;
                    RotateDone                    = 0;
                    PID_Fetch_Pitch_Left.p        = 1500;
                    PID_Fetch_Pitch_Left.d        = 8000;
                }
                // 松开爪子
                if (-1 * pitchLeftAngle >= CHOOSE(40, 50, 60)) {
                    GET_OFF;
                    PID_Fetch_Pitch_Left.p = 350;
                    PID_Fetch_Pitch_Left.d = 3000;
                }
                // 跳转:爪子到位
                if (-1 * pitchLeftAngle >= CHOOSE(40, 50, 60) && RotateDone) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchUnlock) {
                /* 收回爪子 */
                if (lastFetchState != FetchUnlock) {
                    lastFetchState = FetchUnlock;
                    // 转爪子
                    pitchAngleTargetProgress      = 0;
                    pitchAngleTargetProgressDelta = 0.1;
                    pitchAngleTargetStart         = -1 * pitchLeftAngle;
                    pitchAngleTargetStop          = 20;
                    RotateDone                    = 0;
                }
                // 跳转:爪子到位
                if (RotateDone) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchDone) {
                /* 抓取完成 */
                if (remoteData.switchRight != 2) {
                    FetchState = FetchWaitSignal;
                    continue;
                }
                break;
            }
        }

        // 更新斜坡
        if (pitchAngleTargetProgress < 1) {
            pitchAngleTargetProgress += pitchAngleTargetProgressDelta;
        }
        pitchAngleTarget = RAMP(pitchAngleTargetStart, pitchAngleTargetStop, pitchAngleTargetProgress);

        // if (remoteData.ly > 50) {
        //     pitchAngleTarget = 140;
        // } else if (remoteData.ly < -50) {
        //     pitchAngleTarget = 40;
        // }

        // PID_Fetch_Pitch_Left.p  = CHOOSEL(500, 1000, 1500);
        // PID_Fetch_Pitch_Right.p = CHOOSEL(500, 1000, 1500);
        // PID_Fetch_Pitch_Left.i  = CHOOSEL(0, 4, 6);
        // PID_Fetch_Pitch_Right.i = CHOOSEL(0, 4, 6);
        // PID_Fetch_Pitch_Left.d  = CHOOSER(5000, 7500, 10000);
        // PID_Fetch_Pitch_Right.d = CHOOSER(5000, 7500, 10000);

        // PID_Fetch_X.p    = CHOOSEL(10, 30, 100);
        // PID_Fetch_X.d    = CHOOSER(30, 100, 300);

        // 计算PID
        PID_Calculate(&PID_Fetch_X, xSpeedTarget, xSpeed);
        PID_Calculate(&PID_Fetch_Pitch_Left, -1 * pitchAngleTarget, pitchLeftAngle);
        PID_Calculate(&PID_Fetch_Pitch_Right, -1 * pitchLeftAngle, pitchRightAngle);

        // 更新状态量
        RotateDone = pitchAngleTargetProgress >= 1 && ABS(lastPitchLeftAngle - pitchLeftAngle) < 1;

        // 输出电流
        Motor_Fetch_X.input           = PID_Fetch_X.output;
        Motor_Fetch_Left_Pitch.input  = PID_Fetch_Pitch_Left.output;
        Motor_Fetch_Right_Pitch.input = PID_Fetch_Pitch_Right.output;

        // 更新过去值
        lastPitchLeftAngle = pitchLeftAngle;

        // 更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Raise(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    float raiseAngleTarget = 0;
    float raiseProgress    = 0;
    float rampStart        = 0;
    float rampStop         = 0;
    int   lastState        = 0; // 上一次抬升机构位置

    PID_Init(&PID_Raise_Left_Angle, 18, 0.01, 0.3, 340, 180);  // 18 0.015
    PID_Init(&PID_Raise_Left_Speed, 30, 0.1, 0, 10000, 5000);  // 30 1
    PID_Init(&PID_Raise_Right_Angle, 3, 0.02, 0, 290, 180);    // 24 0.018
    PID_Init(&PID_Raise_Right_Speed, 30, 0.5, 0, 10000, 5000); // 35 0.5

    // PID_Raise_Right_Speed.p = CHOOSER(10, 30, 100);

    while (1) {
        if (RaiseMode) {
            if (rampStop != 1000) {
                raiseProgress = 0;
                rampStart     = Motor_Raise_Right.angle;
                rampStop      = 1000;
            }
        } else {
            if (rampStop != 0) {
                raiseProgress = 0;
                rampStart     = Motor_Raise_Right.angle;
                rampStop      = 0;
            }
        }

        // 计算角度斜坡
        if (raiseProgress < 1) {
            raiseProgress += 0.04f;
        }
        raiseAngleTarget = RAMP(rampStart, rampStop, raiseProgress);

        // 主从控制
        PID_Calculate(&PID_Raise_Right_Angle, raiseAngleTarget, Motor_Raise_Right.angle);
        PID_Calculate(&PID_Raise_Right_Speed, PID_Raise_Right_Angle.output, Motor_Raise_Right.speed * RPM2RPS);
        PID_Calculate(&PID_Raise_Left_Angle, -Motor_Raise_Right.angle, Motor_Raise_Left.angle);
        PID_Calculate(&PID_Raise_Left_Speed, PID_Raise_Left_Angle.output, Motor_Raise_Left.speed * RPM2RPS);

        // Motor_Raise_Left.input  = RaiseMode ? PID_Raise_Left_Speed.output : 0;
        // Motor_Raise_Right.input = RaiseMode ? PID_Raise_Right_Speed.output : 0;

        Motor_Raise_Right.input = PID_Raise_Right_Speed.output;
        Motor_Raise_Left.input  = PID_Raise_Left_Speed.output;

        DebugData.debug2 = Motor_Raise_Left.angle;
        DebugData.debug3 = -Motor_Raise_Right.angle;

        DebugData.debug4 = Motor_Raise_Left.speed;
        DebugData.debug5 = PID_Raise_Left_Angle.output;

        DebugData.debug6 = Motor_Pitch.angle;

        // 更新抬升状态量
        FantongRaised = ABS(PID_Raise_Right_Angle.error) < 5 && RaiseMode;
        // FantongRaised = 1;
        vTaskDelayUntil(&LastWakeTime, 5);
    }

    vTaskDelete(NULL);
}

void Task_Rescue(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        if (RescueMode) {
            RESCUE_HOOK_DOWN;
        } else {
            RESCUE_HOOK_UP;
        }
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
        if (KTV_Play(Music_XP)) break;
        vTaskDelayUntil(&LastWakeTime, 150);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 发射机构 (拨弹轮)
 */

void Task_Fire_Stir(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 射击模式
    enum shootMode_e { shootIdle = 0, shootToDeath }; // 停止, 连发
    enum shootMode_e shootMode = shootIdle;

    // 热量控制
    int   shootNum        = 0;
    int   mayShootNum     = 0;
    int   maxBulletSpeed  = 0;
    float lastBulletSpeed = 0;
    float maxShootHeat    = 0;
    int   stirSpeed       = 0;
    int   stirAngle       = 0;
    int   seconds         = 0;

    // 视觉系统
    int16_t lastSeq = 0;

    // PID 初始化
    PID_Init(&PID_StirAngle, 1, 0, 0, 9000, 6000);  // 拨弹轮角度环
    PID_Init(&PID_StirSpeed, 18, 0, 0, 6000, 1000); // 拨弹轮速度环

    // 开启激光
    LASER_ON;

    while (1) {
        // 弹舱盖开关

        // 拨弹速度
        // if (Judge.robotState.robot_level == 1) {
        //     stirSpeed = 110;
        // } else if (Judge.robotState.robot_level == 2) {
        //     stirSpeed = 140;
        // } else if (Judge.robotState.robot_level == 3) {
        //     stirSpeed = 160;
        // }

        stirSpeed = 50;

        // X模式
        if (FastShootMode) {
            stirSpeed *= 2;
        }

        //热量控制
        // maxShootHeat = Judge.robotState.shooter_heat0_cooling_limit * 0.8; // todo: why?
        // maxShootHeat = Judge.robotState.shooter_heat0_cooling_limit - 60;

        // 输入射击模式
        // if (StirEnabled && Judge.powerHeatData.shooter_heat0 < maxShootHeat) {
        //     shootMode = shootToDeath;
        // } else {
        //     shootMode = shootIdle;
        // }
        if (StirEnabled) {
            shootMode = shootToDeath;
        } else {
            shootMode = shootIdle;
        }
        // 视觉辅助
        // if (!PsEnabled) {
        //     lastSeq = Ps.autoaimData.seq;
        // } else if (lastSeq != Ps.autoaimData.seq && Judge.powerHeatData.shooter_heat0 < maxShootHeat) {
        //     lastSeq   = Ps.autoaimData.seq;
        //     shootMode = Ps.autoaimData.biu_biu_state ? shootToDeath : shootIdle;
        // } else {
        //     shootMode = shootIdle;
        // }

        // 控制拨弹轮
        if (shootMode == shootIdle) {
            // 停止
            Motor_Stir.input = 0;
        } else if (shootMode == shootToDeath) {
            // 连发
            PID_Calculate(&PID_StirSpeed, stirSpeed, Motor_Stir.speed * RPM2RPS);
            Motor_Stir.input = PID_StirSpeed.output;
        }
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
    }
    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

    //获得 Stone ID
    BSP_Stone_Id_Init(&Board_Id, &Robot_Id);

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
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
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
    // xTaskCreate(Task_Fetch, "Task_Fetch", 400, NULL, 5, NULL);
    xTaskCreate(Task_Raise, "Task_Raise", 400, NULL, 5, NULL); // 抬升
    // xTaskCreate(Task_Rescue, "Task_Rescue", 400, NULL, 5, NULL); // 抬升
    xTaskCreate(Task_Fire_Stir, "Task_Fire_Stir", 400, NULL, 5, NULL);
    xTaskCreate(Task_Snail, "Task_Snail", 500, NULL, 5, NULL);

    // DMA发送任务
    // xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Vision_Communication, "Task_Vision_Communication", 500, NULL, 6, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 5, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
