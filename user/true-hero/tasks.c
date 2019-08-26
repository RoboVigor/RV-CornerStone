/**
 * @brief 真*英雄
 * @version 1.2.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2) {
            vTaskSuspendAll();
            while (1) {
                Can_Send(CAN1, 0x200, 0, 0, 0, 0);
                Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);
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

        if (remoteData.switchLeft == 1 || remoteData.switchLeft == 3) {
            ControlMode = 1; //遥控器模式
        } else if (remoteData.switchLeft == 2) {
            ControlMode = 2; //键鼠模式
        }
        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Client_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        int      index = 0;
        uint16_t dataLength;
        while (DMA_GetFlagStatus(DMA2_Stream6, DMA_IT_TCIF6) != SET) {
        }
        DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6);
        DMA_Cmd(DMA2_Stream6, DISABLE);
        // 客户端自定义数据
        Judge.clientCustomData.data_cmd_id = Protocol_Interact_Id_Client_Data;
        Judge.clientCustomData.send_id     = Judge.robotState.robot_id;
        Judge.clientCustomData.receiver_id = (Judge.clientCustomData.send_id % 10) | (Judge.clientCustomData.send_id / 10) << 4 | (0x01 << 8);
        Judge.clientCustomData.data1       = 1;
        Judge.clientCustomData.data2       = 1.1;
        Judge.clientCustomData.data3       = 1.11;
        Judge.clientCustomData.masks       = 0;
        Judge.clientCustomData.bit1        = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
        Judge.clientCustomData.bit2        = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
        Judge.clientCustomData.bit3        = PsEnabled;
        Judge.clientCustomData.bit4        = StirStop;
        Judge.clientCustomData.bit5        = ServoEnabled;
        Judge.clientCustomData.bit6        = UpEnabled;
        dataLength                         = Protocol_Pack_Length_0301_Header + Protocol_Pack_Length_0301_Client_Data;
        Protocol_Pack(&Judge, dataLength, Protocol_Interact_Id_Client_Data);
        DMA_SetCurrDataCounter(DMA2_Stream6, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
        DMA_Cmd(DMA2_Stream6, ENABLE);
        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Gimbal(void *Parameters) {

    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 反馈值
    float yawAngle, yawSpeed, pitchAngle, pitchSpeed, chassisAngle;

    // 目标值
    float yawAngleTarget            = 0; // 目标Yaw
    float pitchAngleTarget          = 0; // 目标Pitch
    float yawAngleTargetControl     = 0; // 遥控器输入
    float pitchAngleTargetControl   = 0; // 遥控器输入
    float pitchAngleTargetFix       = 0; // 上坡补偿
    float pitchAngleTargetFixStable = 0; // 上坡补偿
    float yawAngleTargetPs          = 0; // 视觉辅助
    float pitchAngleTargetPs        = 0; // 视觉辅助

    // 视觉系统
    int16_t lastSeq = 0;

    // Pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = Gyroscope_EulerData.roll;
    float pitchAngleTargetRamp = 0;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 20, 0, 0, 3000, 0);
    PID_Init(&PID_Cloud_YawSpeed, 60, 0, 0, 5000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 20, 0, 0, 3000, 0);
    PID_Init(&PID_Cloud_PitchSpeed, 60, 0, 0, 5000, 0);

    while (1) {
        // 重置目标
        yawAngleTarget   = 0;
        pitchAngleTarget = 0;

        // 设置反馈
        yawAngle     = -1 * Gyroscope_EulerData.yaw;    // 逆时针为正
        yawSpeed     = -1 * ImuData.gz / GYROSCOPE_LSB; // 逆时针为正
        pitchAngle   = Gyroscope_EulerData.roll;        // 逆时针为正
        pitchSpeed   = ImuData.gy / GYROSCOPE_LSB;      // 逆时针为正
        chassisAngle = Motor_Pitch.angle + pitchAngle;

        // 遥控器输入角度目标
        if (ControlMode == 1) {
            if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f * 180 * interval;
            if (ABS(remoteData.ry) > 30) pitchAngleTargetControl += -1 * remoteData.ry / 660.0f * 150 * interval;
        } else if (ControlMode == 2) {
            yawAngleTargetControl += mouseData.x * 2 * interval;
            pitchAngleTargetControl += mouseData.y * interval;
        }
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;

        // 视觉辅助
        // if (ControlMode == 1) {
        //     if (remoteData.switchRight == 3) {
        //         PsEnabled = 1;
        //     } else {
        //         PsEnabled = 0;
        //     }
        // }
        if (ControlMode == 2) {
            if (!mouseData.pressRight) {
                PsEnabled = 0;
            } else if (mouseData.pressRight) {
                PsEnabled = 1;
            }
        }
        if (!PsEnabled) {
            lastSeq = Ps.autoaimData.seq;
        } else if (lastSeq != Ps.autoaimData.seq) {
            lastSeq = Ps.autoaimData.seq;
            yawAngleTargetPs += Ps.autoaimData.yaw_angle_diff;
            pitchAngleTargetPs -= Ps.autoaimData.pitch_angle_diff;
        }
        MIAO(pitchAngleTargetPs, GIMBAL_PITCH_MIN - pitchAngleTarget, GIMBAL_PITCH_MAX - pitchAngleTarget);
        yawAngleTarget += yawAngleTargetPs;
        pitchAngleTarget += pitchAngleTargetPs;

        // 上坡补偿
        // pitchAngleTargetFixStable = FirstOrderLowPassFilter(-1 * (chassisAngle / 40.0) * (GIMBAL_PITCH_MIN - pitchAngleTarget), &pitchAngleTargetFix,
        // 200, 20); pitchAngleTarget += pitchAngleTargetFixStable;

        // 限制云台运动范围
        MIAO(pitchAngleTarget, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

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
        Can_Send(CAN1, 0x1FF, PID_Cloud_YawSpeed.output, PID_Cloud_PitchSpeed.output, 0, 0);

        // 调试信息
        // DebugData.debug1 = PsEnabled;
        // DebugData.debug2 = Ps.autoaimData.seq;
        // DebugData.debug3 = ImuData.gz;
        // DebugData.debug4 = autoAimStart;
        // DebugData.debug5 = Ps.autoaimData.seq;
        // DebugData.debug6 = PID_Cloud_PitchSpeed.output;
        // DebugData.debug7 = yawAngle;
        // DebugData.debug8 = pitchAngleTarget;

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
    float vx = 0;
    float vy = 0;
    float vw = 0;
    float targetPower;

    // 反馈值
    float motorAngle;
    float filter[6] = {0, 0, 0, 0, 0, 0};
    int   filterp   = 0;
    float motorSpeedStable;
    float power       = 0;
    float powerBuffer = 0;

    // 底盘跟随PID
    float followDeadRegion = 3.0;
    PID_Init(&PID_Follow_Angle, 0.05, 0, 0, 1000, 0);
    PID_Init(&PID_Follow_Speed, 9, 0, 0, 1000, 1000);

    // 麦轮速度PID
    PID_Init(&PID_LFCM, 18, 0, 0, 15000, 7500);
    PID_Init(&PID_LBCM, 18, 0, 0, 15000, 7500);
    PID_Init(&PID_RBCM, 18, 0, 0, 15000, 7500);
    PID_Init(&PID_RFCM, 18, 0, 0, 15000, 7500);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    //地盘运动斜坡函数
    float xRampProgress = 0;
    float xRampStart    = 0;
    float xTargetRamp   = 0;
    float yRampProgress = 0;
    float yRampStart    = 0;
    float yTargetRamp   = 0;

    while (1) {
        // 设置反馈值
        motorAngle  = Motor_Yaw.angle;                          // 电机角度
        power       = Judge.powerHeatData.chassis_power;        // 裁判系统功率
        powerBuffer = Judge.powerHeatData.chassis_power_buffer; // 裁判系统功率缓冲

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
        if (ControlMode == 1) {
            vx = -remoteData.lx / 660.0f * 4;
            vy = remoteData.ly / 660.0f * 12;
        } else if (ControlMode == 2) {
            xTargetRamp = RAMP(xRampStart, 660, xRampProgress);
            if (xRampProgress < 0.5) {
                xRampProgress += 0.004f;
            } else if (xRampProgress > 0.5 && xRampProgress < 1) {
                xRampProgress += 0.002f;
            }
            yTargetRamp = RAMP(yRampStart, 660, yRampProgress);
            if (yRampProgress < 0.5) {
                yRampProgress += 0.006f;
            } else if (yRampProgress > 0.5 && yRampProgress < 1) {
                yRampProgress += 0.004f;
            }
            MIAO(xRampProgress, 0, 1);
            MIAO(yRampProgress, 0, 1);

            vx = (keyboardData.A - keyboardData.D) * xTargetRamp / 660.0f * 4;
            vy = (keyboardData.W - keyboardData.S) * yTargetRamp / 660.0f * 12;

            if (keyboardData.W == 0 && keyboardData.S == 0) {
                yRampProgress = 0;
                yRampStart    = 0;
            }
            if (keyboardData.A == 0 && keyboardData.D == 0) {
                xRampProgress = 0;
                xRampStart    = 0;
            }
        }

        vw = ABS(PID_Follow_Angle.error) < followDeadRegion ? 0 : (-1 * PID_Follow_Speed.output * DPS2RPS);
        if (ABS(remoteData.rx) > 300 || ABS(mouseData.x) > 41) {
            vy = vy / 5.0f;
            vx = vx / 2.0f;
        }

        // 麦轮解算及限速
        targetPower = 80.0 - WANG(40.0 - ChassisData.powerBuffer, 0, 40) / 40.0 * 80.0; // 设置目标功率
        Chassis_Update(&ChassisData, vx, vy, vw);                                       // 更新麦轮转速
        // Chassis_Fix(&ChassisData, motorAngle);                               // 修正旋转后底盘的前进方向
        Chassis_Calculate_Rotor_Speed(&ChassisData);                                  // 麦轮解算
        Chassis_Limit_Rotor_Speed(&ChassisData, CHASSIS_ROTOR_SPEED);                 // 设置转子速度上限 (rad/s)
        Chassis_Limit_Power(&ChassisData, targetPower, power, powerBuffer, interval); // 根据功率限幅
        if (keyboardData.G && !keyboardData.Ctrl) {
            UpEnabled  = 1;
            PID_LFCM.i = 0.5;
            PID_LBCM.i = 0.5;
            PID_RBCM.i = 0.5;
            PID_RFCM.i = 0.5;
        } else if (keyboardData.G && keyboardData.Ctrl) {
            UpEnabled         = 0;
            PID_LFCM.i        = 0;
            PID_LBCM.i        = 0;
            PID_RBCM.i        = 0;
            PID_RFCM.i        = 0;
            PID_LFCM.output_I = 0;
            PID_LBCM.output_I = 0;
            PID_RBCM.output_I = 0;
            PID_RFCM.output_I = 0;
        }
        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调
        Can_Send(CAN1,
                 0x200,
                 PID_LFCM.output * ChassisData.powerScale,
                 PID_LBCM.output * ChassisData.powerScale,
                 PID_RBCM.output * ChassisData.powerScale,
                 PID_RFCM.output * ChassisData.powerScale);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = ABS(PID_Follow_Angle.error);
        // DebugData.debug2 = -1 * PID_Follow_Speed.output * DPS2RPS * 5 * 1000;
        // DebugData.debug3 = motorSpeedStable;
        // DebugData.debug4 = PID_LFCM.output;
        // DebugData.debug5 = followDeadRegion;
        // DebugData.debug6 = Motor_LF.speed * RPM2RPS;
        // DebugData.debug7 = remoteData.rx;
        // DebugData.debug7 = ChassisData.rotorSpeed[0];
        // DebugData.debug8 = PID_Follow_Speed.output;
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
    //模式
    int   shootMode         = 1; // 0为纯手动控制大拨弹   1为自动控制大拨弹
    int   state             = 0;
    int   counter1          = 0;
    int   counter2          = 0;
    int   counter3          = 0;
    int   stop              = 0;
    int   lastStop          = 0;
    float speedRampProgress = 0;
    float speedRampStart    = 0;
    float speedTargetRamp   = 0;
    int   shootState        = 1;
    int   stirState         = 0;

    //射击参数
    int maxShootHeat = Judge.robotState.shooter_heat1_cooling_limit;

    GPIO_SetBits(GPIOF, GPIO_Pin_0); //微动开关IO口输出高电平

    float rpm2rps = 3.14 / 60;

    // PID 初始化
    PID_Init(&PID_LeftFrictSpeed, 30, 0.1, 0, 10000, 5000);
    PID_Init(&PID_RightFrictSpeed, 30, 0.1, 0, 10000, 5000);
    PID_Init(&PID_Stir2006Speed, 30, 0.1, 0, 5000, 3000);   //半径38
    PID_Init(&PID_Stir3510Speed, 100, 10, 0, 15000, 10000); //半径110

    while (1) {
        speedTargetRamp = RAMP(speedRampStart, 35, speedRampProgress);
        if (speedRampProgress < 1) {
            speedRampProgress += 0.1f;
        }
        if (keyboardData.Q && keyboardData.Ctrl) {
            shootState = 1;
        } else if (keyboardData.Q && !keyboardData.Ctrl) {
            shootState = 2;
        }

        if (ControlMode == 2) {
            if (keyboardData.R && !keyboardData.Ctrl) {
                shootMode = 1;
            } else if (keyboardData.R && keyboardData.Ctrl) {
                shootMode = 0;
            }
        } else if (ControlMode == 1) {
            shootMode = 1;
        }
        // 2006,3510拨弹轮
        if (shootMode) {
            if (ControlMode == 1) {
                if (remoteData.switchLeft == 1) {
                    PID_LeftFrictSpeed.output_I  = 0;
                    PID_RightFrictSpeed.output_I = 0;
                    PID_Calculate(&PID_LeftFrictSpeed, 0, Motor_LeftFrict.speed * rpm2rps);   // 左摩擦轮停止
                    PID_Calculate(&PID_RightFrictSpeed, 0, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮停止
                    LASER_OFF;
                } else if (remoteData.switchLeft == 3) {
                    PID_Calculate(&PID_RightFrictSpeed, 237, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮
                    PID_Calculate(&PID_LeftFrictSpeed, -237, Motor_LeftFrict.speed * rpm2rps);  // 左摩擦轮

                    LASER_ON; // 激光开启
                }
            }
            if (ControlMode == 2) {
                if (shootState == 1) {
                    PID_LeftFrictSpeed.output_I  = 0;
                    PID_RightFrictSpeed.output_I = 0;
                    PID_Calculate(&PID_LeftFrictSpeed, 0, Motor_LeftFrict.speed * rpm2rps);   // 左摩擦轮停止
                    PID_Calculate(&PID_RightFrictSpeed, 0, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮停止
                    LASER_OFF;                                                                // 激光关闭
                } else if (shootState == 2) {
                    PID_Calculate(&PID_RightFrictSpeed, 230, Motor_RightFrict.speed * rpm2rps); // 右摩擦轮
                    PID_Calculate(&PID_LeftFrictSpeed, -230, Motor_LeftFrict.speed * rpm2rps);  // 左摩擦轮
                    LASER_ON;                                                                   // 激光开启
                }

                if (keyboardData.E && !keyboardData.Ctrl) {
                    ServoEnabled = 1;
                    PWM_Set_Compare(&PWM_Magazine_Servo, 10);
                } else if (keyboardData.E && keyboardData.Ctrl) {
                    ServoEnabled = 0;
                    PWM_Set_Compare(&PWM_Magazine_Servo, 17);
                }
            }

            if (!stop) {
                if (ControlMode == 1) {
                    if (remoteData.switchRight == 3 && Judge.powerHeatData.shooter_heat1 < (maxShootHeat - 100)) {
                        PID_Stir2006Speed.p = 30;
                        PID_Calculate(&PID_Stir2006Speed, speedTargetRamp, Motor_Stir2006.speed * rpm2rps);
                        PID_Stir3510Speed.output_I = 0;
                        PID_Stir3510Speed.output   = 0;
                        state                      = 1;
                        stirState                  = 1;
                    } else if (remoteData.switchRight == 1) {
                        state = 0;
                    }
                }
                if (ControlMode == 2) {
                    if (mouseData.pressLeft && Judge.powerHeatData.shooter_heat1 < (maxShootHeat - 100)) {
                        PID_Stir2006Speed.p = 30;
                        PID_Calculate(&PID_Stir2006Speed, speedTargetRamp, Motor_Stir2006.speed * rpm2rps);
                        PID_Stir3510Speed.output_I = 0;
                        PID_Stir3510Speed.output   = 0;
                        state                      = 1;
                        stirState                  = 1;
                    } else if (!mouseData.pressLeft) {
                        state = 0;
                    }
                }
                if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0 && !state) {
                    PID_Calculate(&PID_Stir2006Speed, speedTargetRamp, Motor_Stir2006.speed * rpm2rps);
                    PID_Calculate(&PID_Stir3510Speed, 2.5, Motor_Stir3510.speed * rpm2rps);
                    stirState = 1;
                } else if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 1 && !state) {
                    PID_Stir2006Speed.output_I = 0;
                    PID_Stir3510Speed.output_I = 0;
                    PID_Stir2006Speed.p        = 60;
                    PID_Calculate(&PID_Stir2006Speed, 0, Motor_Stir2006.speed * rpm2rps);
                    PID_Calculate(&PID_Stir3510Speed, 0, Motor_Stir3510.speed * rpm2rps);
                    speedRampProgress = 0;
                    stirState         = 0;
                }
            }
        }

        // if (shootMode == 0) {
        //     //键鼠模式才可启动纯手动控制
        //     if (keyboardData.Z == 1 && keyboardData.Ctrl == 0) {
        //         PID_Calculate(&PID_Stir3510Speed, 4, Motor_Stir3510.speed * rpm2rps);
        //     } else if (keyboardData.Z == 1 && keyboardData.Ctrl == 1) {
        //         PID_Calculate(&PID_Stir3510Speed, 0, Motor_Stir3510.speed * rpm2rps);
        //     }
        //     if (mouseData.pressLeft == 1 && Judge.powerHeatData.shooter_heat1 < (maxShootHeat - 100)) {
        //         PID_Calculate(&PID_Stir2006Speed, speedTargetRamp, Motor_Stir2006.speed * rpm2rps);
        //         PID_Stir3510Speed.output_I = 0;
        //         PID_Stir3510Speed.output   = 0;
        //     } else if (mouseData.pressLeft == 0) {
        //         PID_Calculate(&PID_Stir2006Speed, 0, Motor_Stir2006.speed * rpm2rps);
        //         speedRampProgress = 0;
        //     }
        // }

        //堵转检测
        // if (PID_Stir2006Speed.output > 3000) {
        //     stop = 1;
        // } else {
        //     stop = 0;
        // }
        if (stirState && ABS(Motor_Stir2006.speed * rpm2rps) < 10 && counter3 < 100) {
            counter3 += 1;
        } else if (stirState && PID_Stir2006Speed.output > -200) {
            counter3 = 0;
        }

        if (counter3 >= 100) {
            stop = 1;
        } else {
            stop = 0;
        }

        if (stop && counter1 < 50) {
            counter1 += 1;
            PID_Stir2006Speed.output = -1000;
            PID_Stir3510Speed.output = 0;
        }
        if (counter1 >= 50) {
            counter1 = 0;
            counter3 = 0;
        }

        StirStop = stop;
        Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, PID_Stir2006Speed.output, PID_Stir3510Speed.output);

        vTaskDelayUntil(&LastWakeTime, 10);

        // DebugData.debug1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
        // DebugData.debug2 = counter1;
        // DebugData.debug3 = counter2;
        // DebugData.debug4 = counter3;
        // DebugData.debug5 = counter1;
        // DebugData.debug6 = stirState;
        // DebugData.debug7 = keyboardData.C;
        // DebugData.debug8 = keyboardData.Z;
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

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    xTaskCreate(Task_Fire, "Task_Fire", 400, NULL, 5, NULL);

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 4, NULL);

    // 通讯
    xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}