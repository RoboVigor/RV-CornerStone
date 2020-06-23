/**
 * @brief 工程
 * @version 1.2.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2 && remoteData.switchLeft == 1) {
            vTaskSuspendAll();
            while (1) {
                Can_Send(CAN1, 0x200, 0, 0, 0, 0);
                Can_Send(CAN1, 0x1ff, 0, 0, 0, 0);
                Can_Send(CAN2, 0x200, 0, 0, 0, 0);
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

    RescueMode = 0;
    while (1) {
        FetchMode   = remoteData.switchLeft == 1;
        RaiseMode   = remoteData.switchLeft == 1;
        ChassisMode = remoteData.switchLeft != 1;
        if (remoteData.switchLeft == 2 && remoteData.switchRight == 1) {
            RescueMode = 1;
        } else if (remoteData.switchLeft == 2 && remoteData.switchRight == 2) {
            RescueMode = 0;
        }

        // 调试视觉用
        // PsAimEnabled   = (remoteData.switchLeft == 1) && (remoteData.switchRight != 3);
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 运动模式
    int   mode           = 2; // 底盘运动模式,1直线,2转弯
    int   lastMode       = 2; // 上一次的运动模式
    float yawAngleTarget = 0; // 目标值
    float yawAngle, yawSpeed; // 反馈值

    // 底盘运动
    float vx = 0;
    float vy = 0;
    float vw = 0;

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, 32, 0.1, 0, 12000, 4000);
    PID_Init(&PID_LBCM, 32, 0.1, 0, 12000, 4000);
    PID_Init(&PID_RBCM, 32, 0.1, 0, 12000, 4000);
    PID_Init(&PID_RFCM, 32, 0.1, 0, 12000, 4000);

    // 初始化航向角角度PID和角速度PID
    PID_Init(&PID_YawAngle, 5, 0, 0, 1000, 1000);
    PID_Init(&PID_YawSpeed, 10, 0, 0, 4000, 1000);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    while (1) {

        // 更新运动模式
        mode = ABS(remoteData.rx) < 5 ? 1 : 2;

        // 设置反馈值
        yawAngle = Gyroscope_EulerData.yaw;    // 航向角角度反馈
        yawSpeed = ImuData.gz / GYROSCOPE_LSB; // 逆时针为正

        // 根据控制模式设定运动
        if (ChassisMode) {
            vx = -remoteData.lx / 660.0f * 4;
            vy = remoteData.ly / 660.0f * 2;
            vw = PID_YawSpeed.output / 4000.0f * 6;
        } else {
            vx = 0;
            vy = 0;
            vw = 0;
        }

        // 切换运动模式
        if (mode != lastMode) {
            PID_YawAngle.output_I = 0;        // 清空角度PID积分
            PID_YawSpeed.output_I = 0;        // 清空角速度PID积分
            yawAngleTarget        = yawAngle; // 更新角度PID目标值
            lastMode              = mode;     // 更新lastMode
        }

        // 根据运动模式计算PID
        if (mode == 1) {
            PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngle);      // 计算航向角角度PID
            PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeed); // 计算航向角角速度PID
        } else {
            PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeed); // 计算航向角角速度PID
        }

        // 设置底盘总体移动速度
        Chassis_Update(&ChassisData, vx, vy, vw);

        // 麦轮解算
        Chassis_Calculate_Rotor_Speed(&ChassisData);

        //稍微限制移动速度
        Chassis_Limit_Rotor_Speed(&ChassisData, 800);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调
        Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

        // 调试信息
        // DebugData.debug1 = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin);
        // DebugData.debug2 = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_1);

        // 底盘运动更新频率
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
        DebugData.debug5 = ABS(lastPitchLeftAngle - pitchLeftAngle) * 1000;

        // 计算PID
        PID_Calculate(&PID_Fetch_X, xSpeedTarget, xSpeed);
        PID_Calculate(&PID_Fetch_Pitch_Left, -1 * pitchAngleTarget, pitchLeftAngle);
        PID_Calculate(&PID_Fetch_Pitch_Right, -1 * pitchLeftAngle, pitchRightAngle);

        // 更新状态量
        RotateDone = pitchAngleTargetProgress >= 1 && ABS(lastPitchLeftAngle - pitchLeftAngle) < 1;

        // 输出电流
        Can_Send(CAN2, 0x200, PID_Fetch_X.output, PID_Fetch_Pitch_Left.output, PID_Fetch_Pitch_Right.output, 0);

        DebugData.debug1 = ABS(lastPitchLeftAngle - pitchLeftAngle) * 1000;
        DebugData.debug2 = FetchState * 100;
        DebugData.debug3 = PID_Fetch_Pitch_Left.target * -100;
        DebugData.debug4 = PID_Fetch_Pitch_Left.feedback * -100;
        // DebugData.debug5 = PID_Fetch_X.target;
        DebugData.debug6 = PID_Fetch_X.feedback;
        DebugData.debug7 = PID_Fetch_X.output;
        DebugData.debug8 = PID_Fetch_X.output_D;

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

    PID_Init(&PID_Raise_Left_Angle, 18, 0.015, 0, 340, 170);    // 18 0.015
    PID_Init(&PID_Raise_Left_Speed, 30, 1, 0, 10000, 5000);     // 30 1
    PID_Init(&PID_Raise_Right_Angle, 24, 0.02, 0, 290, 145);    // 24 0.018
    PID_Init(&PID_Raise_Right_Speed, 40, 0.85, 0, 10000, 5000); // 35 0.5

    while (1) {
        if (RaiseMode) {
            if (rampStop != 375) {
                raiseProgress = 0;
                rampStart     = Motor_Raise_Left.angle;
                rampStop      = 375;
            }
        } else {
            if (rampStop != 0) {
                raiseProgress = 0;
                rampStart     = Motor_Raise_Left.angle;
                rampStop      = 0;
            }
        }

        // 计算角度斜坡
        if (raiseProgress < 1) {
            raiseProgress += 0.05f;
        }
        raiseAngleTarget = RAMP(rampStart, rampStop, raiseProgress);

        // 主从控制
        PID_Calculate(&PID_Raise_Left_Angle, raiseAngleTarget, Motor_Raise_Left.angle);
        PID_Calculate(&PID_Raise_Left_Speed, PID_Raise_Left_Angle.output, Motor_Raise_Left.speed * RPM2RPS);
        PID_Calculate(&PID_Raise_Right_Angle, -Motor_Raise_Left.angle, Motor_Raise_Right.angle);
        PID_Calculate(&PID_Raise_Right_Speed, PID_Raise_Right_Angle.output, Motor_Raise_Right.speed * RPM2RPS);

        Can_Send(CAN1, 0x1FF, PID_Raise_Left_Speed.output, PID_Raise_Right_Speed.output, 0, 0);

        // 更新抬升状态量
        // FantongRaised = ABS(PID_Raise_Left_Angle.error) < 5 && RaiseMode;
        FantongRaised = 1;
        vTaskDelayUntil(&LastWakeTime, 10);
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

// void Task_Client_Communication(void *Parameters) {
//     TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
//     float      interval     = 0.1;                 // 任务运行间隔 s
//     int        intervalms   = interval * 1000;     // 任务运行间隔 ms

//     while (1) {
//         int      index = 0;
//         uint16_t dataLength;

//         while (DMA_GetFlagStatus(DMA2_Stream6, DMA_IT_TCIF6) != SET) {
//         }
//         DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);
//         DMA_Cmd(DMA2_Stream6, DISABLE);

//         switch (Judge.mode) {
//         case MODE_CLIENT_DATA: {
//             // 客户端自定义数据
//             Judge.clientCustomData.data_cmd_id = Protocol_Interact_Id_Client_Data;
//             Judge.clientCustomData.send_id     = Judge.robotState.robot_id;
//             Judge.clientCustomData.receiver_id = (Judge.clientCustomData.send_id % 10) | (Judge.clientCustomData.send_id / 10) << 4 | (0x01 << 8);

//             Judge.clientCustomData.data1 = 1;
//             Judge.clientCustomData.data2 = 1.1;
//             Judge.clientCustomData.data3 = 1.11;
//             Judge.clientCustomData.masks = 0x3c;

//             dataLength = Protocol_Pack_Length_0301_Header + Protocol_Pack_Length_0301_Client_Data;

//             Protocol_Pack(&Judge, dataLength, Protocol_Interact_Id_Client_Data);

//             Judge.mode = MODE_ROBOT_INTERACT;
//         } break;

//         case MODE_ROBOT_INTERACT: {
//             // 机器人间通信
//             Judge.robotInteractiveData[0].data_cmd_id = 0x0200;
//             Judge.robotInteractiveData[0].send_id     = Judge.robotState.robot_id;
//             Judge.robotInteractiveData[0].receiver_id = 1;

//             Judge.robotInteractiveData[0].transformer[index++].F = 1;
//             Judge.robotInteractiveData[0].transformer[index++].F = 1.1;
//             Judge.robotInteractiveData[0].transformer[index++].F = 1.11;
//             Judge.robotInteractiveData[0].transformer[index++].F = 1.111;

//             dataLength = Protocol_Pack_Length_0301_Header + index * sizeof(float);

//             Protocol_Pack(&Judge, dataLength, 0x0200);

//             Judge.mode = MODE_CLIENT_GRAPH;
//         } break;

//         case MODE_CLIENT_GRAPH: {
//             // 客户端自定义图形
//             Judge.clientGraphicDraw.data_cmd_id = Protocol_Interact_Id_Client_Graph;
//             Judge.clientGraphicDraw.send_id     = Judge.robotState.robot_id;
//             Judge.clientGraphicDraw.receiver_id = (Judge.clientCustomData.send_id % 10) | (Judge.clientCustomData.send_id / 10) << 4 | (0x01 << 8);

//             Judge.clientGraphicDraw.operate_tpye = 1; // 0:空操作 1:增加图形 2:修改图形 3:删除单个图形 5:删除一个图层 6:删除所有图形
//             Judge.clientGraphicDraw.graphic_tpye = 3; // 0:空形 1:直线 2:矩形 3:正圆 4:椭圆 5:弧形 6:文本（ASCII 字码）
//             Judge.clientGraphicDraw.layer        = 5; // 图层0-9
//             Judge.clientGraphicDraw.width        = 4; // 线宽
//             Judge.clientGraphicDraw.color        = 4; // 官方 0:红/蓝 1:黄 2:绿 3:橙 4:紫 5:粉 6:青 7:黑 8:白
//                                                       // 自测 0:红 1:橙 2:黄 3:绿 4:青 5:蓝 6:紫 7:粉 8:黑

//             Judge.clientGraphicDraw.graphic_name[0] = 0;
//             Judge.clientGraphicDraw.graphic_name[1] = 0;
//             Judge.clientGraphicDraw.graphic_name[2] = 0;
//             Judge.clientGraphicDraw.graphic_name[3] = 0;
//             Judge.clientGraphicDraw.graphic_name[4] = 1;

//             Judge.clientGraphicDraw.start_x = 960;
//             Judge.clientGraphicDraw.start_y = 540;
//             Judge.clientGraphicDraw.radius  = 100;

//             dataLength = Protocol_Pack_Length_0301_Header + Protocol_Pack_Length_0301_Client_Graph;

//             Protocol_Pack(&Judge, dataLength, Protocol_Interact_Id_Client_Graph);

//             Judge.mode = MODE_CLIENT_DATA;
//         } break;

//         default:
//             break;
//         }

//         DMA_SetCurrDataCounter(DMA2_Stream6, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
//         DMA_Cmd(DMA2_Stream6, ENABLE);

//         // 发送频率
//         vTaskDelayUntil(&LastWakeTime, intervalms);

//         // 调试信息
//         // DebugData.debug1 = Judge.robotInteractiveData[1].transformer[0].F * 1000;
//         // DebugData.debug2 = Judge.robotInteractiveData[1].transformer[1].F * 1000;
//         // DebugData.debug3 = Judge.robotInteractiveData[1].transformer[2].F * 1000;
//         // DebugData.debug4 = Judge.robotInteractiveData[1].transformer[3].F * 1000;
//     }
//     vTaskDelete(NULL);
// }

void Task_Vision_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        int      index = 0;
        uint16_t dataLength;

        while (DMA_GetFlagStatus(DMA1_Stream0, DMA_IT_TCIF0) != SET) {
        }
        DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0);
        DMA_Cmd(DMA1_Stream0, DISABLE);

        // 视觉通信
        Ps.visionInteractiveData.transformer[index].U16[1]   = 0x6666;
        Ps.visionInteractiveData.transformer[index++].U16[2] = 0x6666;

        dataLength = index * sizeof(float);

        Protocol_Pack(&Ps, dataLength, Protocol_Interact_Id_Vision);

        DMA_SetCurrDataCounter(DMA1_Stream0, index);
        DMA_Cmd(DMA1_Stream0, ENABLE);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Debug_Magic_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        printf("Yaw: %f \r\n", Gyroscope_EulerData.yaw);
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
        if (KTV_Play(Music_XP)) break;
        vTaskDelayUntil(&LastWakeTime, 150);
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
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    // xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    xTaskCreate(Task_Fetch, "Task_Fetch", 400, NULL, 3, NULL);
    xTaskCreate(Task_Raise, "Task_Raise", 400, NULL, 3, NULL);   // 抬升
    xTaskCreate(Task_Rescue, "Task_Rescue", 400, NULL, 3, NULL); // 抬升

    // DMA发送任务
    // xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Vision_Communication, "Task_Vision_Communication", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
