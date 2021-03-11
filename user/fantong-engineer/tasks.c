/**
 * @brief 工程
 * @version 1.2.0
 */
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      interval     = 0.01;            // 任务运行间隔 s
    int        intervalms   = interval * 1000; // 任务运行间隔 ms
    while (1) {
        MilkMode     = LEFT_SWITCH_TOP && RIGHT_SWITCH_BOTTOM;
        FetchMode    = LEFT_SWITCH_TOP + ((LEFT_SWITCH_TOP && RIGHT_SWITCH_TOP) ? 1 : 0);
        RaiseMode    = (LEFT_SWITCH_MIDDLE && RIGHT_SWITCH_TOP) ? 1 : ((LEFT_SWITCH_MIDDLE && RIGHT_SWITCH_BOTTOM) ? 0 : RaiseMode);
        FrictEnabled = LEFT_SWITCH_BOTTOM && (RIGHT_SWITCH_TOP || RIGHT_SWITCH_MIDDLE);
        StirEnabled  = LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_TOP;
        SafetyMode   = LEFT_SWITCH_BOTTOM && RIGHT_SWITCH_BOTTOM;
        ChassisMode  = !LEFT_SWITCH_TOP;
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Board_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    uint16_t id;         // 通讯ID
    uint16_t dataLength; // 数据长度

    while (1) {

        // 板间通信
        if (BOARD_CHASSIS) {
            // id = 0x501;
            // ProtocolData.fetch.fetchMode = FetchMode;
            // ProtocolData.fetch.milkMode  = MilkMode;
        } else if (BOARD_FETCH) {
            ProtocolData.chassis.raiseMode         = RaiseMode;
            ProtocolData.chassis.gimbalVelocityYaw = 1.123;
            Bridge_Send_Protocol_Once(&Node_Board, 0x502);
        }

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Remote_Share(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms
    int        i;

    uint16_t id;         // 通讯ID
    uint16_t dataLength; // 数据长度

    while (1) {
        // if (remoteShareHost) {
        //     // host: 打包遥控器数据
        //     id = 0x501;
        //     for (i = 0; i < 19; i++) {
        //         ProtocolData.remote.data[i] = remoteBuffer[i];
        //     }
        //     DMA_Disable(UART7_Tx);
        //     dataLength = Protocol_Pack(&Node_Board, id);
        //     DMA_Enable(UART7_Tx, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
        // } else if (remoteShareClient) {
        //     // client: 更新遥控器数据
        //     DBus_Update(&remoteData, &keyboardData, &mouseData, ProtocolData.remote.data);
        // } else {
        //     // client: 等待遥控器数据
        //     for (i = 0; i < 19; i++) {
        //         if (ProtocolData.remote.data[i] != 0) {
        //             remoteShareClient = 1;
        //             continue;
        //         }
        //     }
        // }

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
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
        vx = -remoteData.lx / 660.0f * 8.0;
        vy = remoteData.ly / 660.0f * 12.0;
        vx += ProtocolData.chassis.chassisVelocityX;
        vy += ProtocolData.chassis.chassisVelocityY;

        vw = ABS(PID_Follow_Angle.error) < followDeadRegion ? 0 : (-1 * PID_Follow_Speed.output * DPS2RPS);

        if (!ChassisMode) {
            vx = 0;
            vx = 0;
            vw = 0;
        }
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
        Motor_LF.input = PID_LFCM.output;
        Motor_LB.input = PID_LBCM.output;
        Motor_RB.input = PID_RBCM.output;
        Motor_RF.input = PID_RFCM.output;

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

    // 上位机通控制
    int16_t lastSeq = -1;

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
    PID_Init(&PID_Cloud_YawAngle, 10, 0, 0, 15000, 10);
    PID_Init(&PID_Cloud_YawSpeed, 100, 0, 0, 12000, 0);
    // PID_Init(&PID_Cloud_PitchAngle, 10, 0, 0, 16000, 0);
    // PID_Init(&PID_Cloud_PitchSpeed, 10, 0, 0, 16000, 0);
    // PID_Init(&PID_Cloud_YawAngle, 20, 0, 0, 16000, 0);
    // PID_Init(&PID_Cloud_YawSpeed, 2, 0, 0, 4000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 40, 0, 0, 15000, 0);
    PID_Init(&PID_Cloud_PitchSpeed, 1, 0, 0, 1500, 0);

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
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f * 360 * interval;
        if (ABS(remoteData.ry) > 30) pitchAngleTargetControl -= remoteData.ry / 660.0f * 360 * interval;
        yawAngleTargetControl += mouseData.x * 0.5 * 0.005; // 0.005
        pitchAngleTargetControl += mouseData.y * 0.005;
        MIAO(pitchAngleTargetControl, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;

        if (Node_Board.receiveSeq != lastSeq) {
            lastSeq = Node_Board.receiveSeq;
            yawAngleTargetControl += ProtocolData.chassis.gimbalVelocityYaw;
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
        Motor_Yaw.input   = -1 * PID_Cloud_YawSpeed.output;
        Motor_Pitch.input = -1 * PID_Cloud_PitchSpeed.output;
        //任务间隔
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
    int   lastSeq        = -1;
    float xSpeedTargetPs = 0;

    // 临时状态
    int16_t timePassed     = 0;
    uint8_t lastFetchState = -1;

    // 初始化PID
    PID_Init(&PID_Fetch_X, 30, 0.3, 40, 6000, 2000);
    PID_Init(&PID_Fetch_Pitch_Left, 350, 0, 5000, 12000, 1500);
    PID_Init(&PID_Fetch_Pitch_Right, 350, 0, 5000, 12000, 1500);

    // 初始化状态
    FetchState = 0;
    RotateDone = 0;

    while (1) {
        // 设置反馈
        xSpeed          = Motor_Fetch_X.speed * RPM2RPS;
        pitchLeftAngle  = Motor_Fetch_Left_Pitch.angle;
        pitchRightAngle = Motor_Fetch_Right_Pitch.angle;

        // 抓取状态更新
        while (1) {
            if (FetchState == FetchReset) {
                /* 回到默认位置 */
                if (lastFetchState != FetchReset) {
                    lastFetchState = FetchState;
                    // 爪子归位
                    pitchAngleTargetProgress      = 1;
                    pitchAngleTargetProgressDelta = 1;
                    pitchAngleTargetStart         = pitchLeftAngle;
                    pitchAngleTargetStop          = 0;
                }
                // 跳转:爪子和平台已归位
                if (RotateDone) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchWaitRaise) {
                /* 等待抬升 */
                if (1) {
                    // if (ProtocolData.fetch.chassisRaised) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchWaitSignal) {
                /* 平台运动,等待抓取信号 */

                // 遥控器控制左右移动
                if (FetchMode == 1) {
                    if (remoteData.lx > 30) {
                        xSpeedTarget = -200;
                    } else if (remoteData.lx < -30) {
                        xSpeedTarget = 200;
                    } else if (lastSeq != Node_Board.receiveSeq) {
                        xSpeedTarget = ProtocolData.fetch.fetchVelocity;
                        lastSeq      = Node_Board.receiveSeq;
                    }
                } else {
                    xSpeedTarget = 0;
                }
                if (xSpeedTarget == 0 || ABS(Motor_Fetch_X.speed) > 0) {
                    timer = xTaskGetTickCount();
                }
                timePassed = xTaskGetTickCount() - timer;
                if (timePassed > 100) {
                    ProtocolData.chassis.chassisVelocityY = 0.15 * xSpeedTarget / ABS(xSpeedTarget);
                } else {
                    ProtocolData.chassis.chassisVelocityY = 0;
                }

                // 开始抓取
                if (ProtocolData.fetch.fetchMode || FetchMode == 2) {
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
                    pitchAngleTargetStart         = pitchRightAngle;
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
                if (LOCK_STATE == 0) {
                    LOCK_ON;
                    // 重置计时器
                    timer = xTaskGetTickCount();
                }
                timePassed = xTaskGetTickCount() - timer;
                // 跳转:计时器
                if (timePassed >= 1000) {
                    FetchState++;
                    continue;
                }
                break;
            } else if (FetchState == FetchRotateIn) {
                /* 转回来 */
                if (lastFetchState != FetchRotateIn) {
                    lastFetchState = FetchRotateIn;
                    // 转爪子
                    pitchAngleTargetProgress      = 0;
                    pitchAngleTargetProgressDelta = 0.002;
                    pitchAngleTargetStart         = pitchRightAngle;
                    pitchAngleTargetStop          = 30;
                    RotateDone                    = 0;
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
                    pitchAngleTargetProgress      = 0;
                    pitchAngleTargetProgressDelta = 0.1;
                    pitchAngleTargetStart         = pitchRightAngle;
                    pitchAngleTargetStop          = 160;
                    RotateDone                    = 0;
                    // PID_Fetch_Pitch_Left.p        = 1500;
                    // PID_Fetch_Pitch_Left.d        = 8000;
                }
                // 松开爪子
                if (pitchRightAngle >= 20) {
                    LOCK_OFF;
                    // PID_Fetch_Pitch_Left.p = 350;
                    // PID_Fetch_Pitch_Left.d = 3000;
                }
                // 跳转:爪子到位
                if (pitchRightAngle >= 50 && RotateDone) {
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
                    pitchAngleTargetStart         = pitchRightAngle;
                    pitchAngleTargetStop          = 0;
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
                FetchState = FetchWaitSignal;
                break;
            }
        }

        // 更新斜坡
        if (pitchAngleTargetProgress < 1) {
            pitchAngleTargetProgress += pitchAngleTargetProgressDelta;
        }
        pitchAngleTarget = RAMP(pitchAngleTargetStart, pitchAngleTargetStop, pitchAngleTargetProgress);
        // pitchAngleTarget = 20 + remoteData.ry / 6.0;

        // 测试爪子
        // if (remoteData.rx > 50) {
        //     pitchAngleTarget = 170;
        // } else if (remoteData.rx < 50) {
        //     pitchAngleTarget = 20;
        // }

        // //测试GO
        // if (remoteData.ry > 50) {
        //     GO_OUT;
        // } else if (remoteData.ry < -50) {
        //     GO_BACK;
        // }

        // //测试LOCK
        // if (remoteData.ly > 50) {
        //     LOCK_ON;
        // } else if (remoteData.ly < -50) {
        //     LOCK_OFF;
        // }

        // PID_Fetch_Pitch_Left.d  = CHOOSER(3000, 4000, 5000);
        // PID_Fetch_Pitch_Right.d = PID_Fetch_Pitch_Left.d;

        // DebugData.debug1 = timePassed;
        // DebugData.debug2 = ProtocolData.chassis.chassisVelocityX;
        // DebugData.debug3 = ProtocolData.chassis.chassisVelocityY;
        // DebugData.debug4 = Motor_Fetch_X.speed;
        // DebugData.debug5 = xSpeedTarget;
        // DebugData.debug6 = xSpeedTarget == 0 || Motor_Fetch_X.speed > 0;
        // 计算PID
        PID_Calculate(&PID_Fetch_X, xSpeedTarget, xSpeed);
        PID_Calculate(&PID_Fetch_Pitch_Left, -1 * pitchAngleTarget, pitchLeftAngle);
        PID_Calculate(&PID_Fetch_Pitch_Right, pitchAngleTarget, pitchRightAngle);

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

void Task_Milk(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms
    TickType_t timer        = xTaskGetTickCount();

    // 控制
    uint8_t motorInit  = 0;
    int16_t milkTarget = 0;

    // 初始化PID
    PID_Init(&PID_Milk_Angle, 7, 0, 0, 4000, 0);
    PID_Init(&PID_Milk_Speed, 3, 0, 0, 4000, 0);

    while (1) {

        if (!motorInit) {
            Motor_Milk.input = 1000;
            if (Motor_Milk.torque > 10) {
                motorInit               = 1;
                Motor_Milk.round        = 0;
                Motor_Milk.positionBias = Motor_Milk.position;
            }
            continue;
        }
        milkTarget = MilkMode ? 0 : -185;

        PID_Calculate(&PID_Milk_Angle, milkTarget, Motor_Milk.angle);
        PID_Calculate(&PID_Milk_Speed, PID_Milk_Angle.output, Motor_Milk.speed * RPM2RPS);
        Motor_Milk.input = PID_Milk_Speed.output;

        // 更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Go(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms
    while (1) {
        if (ProtocolData.fetch.goMode) {
            GO_OUT;
        } else {
            GO_BACK;
        }

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

    while (1) {
        if (RaiseMode) {
            if (rampStop < 1100) {
                raiseProgress = 0;
                rampStart     = Motor_Raise_Right.angle;
                rampStop      = 1100;
            }
        } else {
            if (rampStop != 0) {
                raiseProgress = 0;
                rampStart     = Motor_Raise_Right.angle;
                rampStop      = 0;
            }
        }

        rampStop += remoteData.ry / 660.0f * 10;

        // 计算角度斜坡
        if (raiseProgress < 1) {
            raiseProgress += 0.04f;
        }
        raiseAngleTarget = RAMP(rampStart, rampStop, raiseProgress);

        PID_Raise_Left_Speed.i = CHOOSER(0.1, 0.3, 0.5);

        // 主从控制
        PID_Calculate(&PID_Raise_Right_Angle, raiseAngleTarget, Motor_Raise_Right.angle);
        PID_Calculate(&PID_Raise_Right_Speed, PID_Raise_Right_Angle.output, Motor_Raise_Right.speed * RPM2RPS);
        PID_Calculate(&PID_Raise_Left_Angle, -Motor_Raise_Right.angle, Motor_Raise_Left.angle);
        PID_Calculate(&PID_Raise_Left_Speed, PID_Raise_Left_Angle.output, Motor_Raise_Left.speed * RPM2RPS);

        Motor_Raise_Right.input = PID_Raise_Right_Speed.output;
        Motor_Raise_Left.input  = PID_Raise_Left_Speed.output;

        // 更新抬升状态量
        FantongRaised = ABS(PID_Raise_Right_Angle.error) < 5 && RaiseMode;
        vTaskDelayUntil(&LastWakeTime, 5);

        DebugData.debug1 = raiseAngleTarget;
        DebugData.debug2 = Motor_Raise_Right.angle;
        DebugData.debug3 = -Motor_Raise_Right.angle;
        DebugData.debug4 = Motor_Raise_Left.angle;
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
