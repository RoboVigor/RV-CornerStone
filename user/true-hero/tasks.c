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

void Task_Gimbal(void *Parameters) {

    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 反馈值
    float yawAngle, yawSpeed, pitchAngle, pitchSpeed, chassisAngle;

    // 目标值
    float yawAngleTarget            = 0;  // 目标Yaw
    float pitchAngleTarget          = 16; // 目标Pitch 0
    float yawAngleTargetControl     = 0;  // 遥控器输入
    float pitchAngleTargetControl   = 0;  // 遥控器输入
    float pitchAngleTargetFix       = 0;  // 上坡补偿
    float pitchAngleTargetFixStable = 0;  // 上坡补偿
    float yawAngleTargetPs          = 0;  // 视觉辅助
    float pitchAngleTargetPs        = 0;  // 视觉辅助

    // 视觉系统
    int16_t lastSeq = 0;

    // Pitch轴斜坡参数
    float pitchRampProgress = 0;
    // float pitchRampStart       = -Gyroscope_EulerData.pitch;
    float pitchRampStart       = Motor_Pitch.angle;
    float pitchAngleTargetRamp = 0;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 42, 0, 0, 3000, 0);  // 35  45
    PID_Init(&PID_Cloud_YawSpeed, 75, 0, 0, 16000, 0); // 65  75
    // PID_Init(&PID_Cloud_PitchAngle, 24, 0, 0, 1000, 0);   //  12 15 18 11 15  13 6 7.5 9
    // PID_Init(&PID_Cloud_PitchSpeed, 105, 0, 0, 16000, 0); //  126 120 115 125 85 100 120 125 130
    PID_Init(&PID_Cloud_PitchAngle, 32, 0, 0, 1000, 0);   //  20
    PID_Init(&PID_Cloud_PitchSpeed, 100, 0, 0, 16000, 0); //  110

    //滤波函数参数
    float   filter_k       = 0.03; //滤波系数
    float   lastpitchAngle = 0;    //上一次pitch反馈
    uint8_t lastFilterFlag = 0;    //上一次角度变化方向
    uint8_t filterFlag     = 0;    //角度变化方向

    while (1) {
        // 重置目标
        yawAngleTarget   = 0;
        pitchAngleTarget = 16; // 0

        // 设置反馈
        yawAngle     = -Gyroscope_EulerData.yaw;    // 逆时针为正
        yawSpeed     = -ImuData.gz / GYROSCOPE_LSB; // 逆时针为正
        pitchAngle   = -Gyroscope_EulerData.pitch;  // 向上为负
        pitchSpeed   = ImuData.gx / GYROSCOPE_LSB;  // 向上为正
        chassisAngle = Motor_Pitch.angle + pitchAngle;

        //设置角度变化方向
        filterFlag = (pitchAngle - lastpitchAngle) > 0 ? 1 : 0; // 1为正方向

        // 设置角度目标
        // if (lastFilterFlag == filterFlag) {
        // if (ControlMode == 1) {
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f;
        if (ABS(remoteData.ry) > 30) pitchAngleTargetControl += remoteData.ry / 660.0f;
        // } else if (ControlMode == 2) {
        //     yawAngleTargetControl += mouseData.x * 2 * interval;
        //     pitchAngleTargetControl += mouseData.y * interval;
        // }
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;
        // } else {
        //     pitchAngleTarget = filter_k * pitchAngle + (1 - filter_k) * lastpitchAngle;
        // }
        // lastpitchAngle = pitchAngle;
        // lastFilterFlag = filterFlag;

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
        // pitchAngleTargetFixStable = FirstOrderLowPassFilter(-1 * (chassisAngle
        // / 40.0) * (GIMBAL_PITCH_MIN - pitchAngleTarget), &pitchAngleTargetFix,
        // 200, 20); pitchAngleTarget += pitchAngleTargetFixStable;

        // 限制云台运动范围
        MIAO(pitchAngleTarget, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

        // 开机时pitch轴匀速抬起
        pitchAngleTargetRamp = RAMP(pitchRampStart, pitchAngleTarget, pitchRampProgress);
        if (pitchRampProgress < 1) {
            pitchRampProgress += 0.01f;
        }

        // 计算PID
        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);

        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTargetRamp, -Motor_Pitch.angle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchSpeed); //正输出向上

        // 输出电流
        Can_Send(CAN2, 0x1FF, 0, PID_Cloud_PitchSpeed.output, 0, 0);
        Can_Send(CAN1, 0x200, 0, 0, PID_Cloud_YawSpeed.output, 0);

        // 调试信息
        // DebugData.debug1 = yawAngle;
        // DebugData.debug2 = yawSpeed;
        // DebugData.debug3 = PID_Cloud_YawAngle.output;
        // DebugData.debug4 = PID_Cloud_YawSpeed.output;
        // DebugData.debug5 = yawAngleTarget;
        // DebugData.debug6 = PID_Cloud_YawAngle.error;
        // DebugData.debug7 = PID_Cloud_YawAngle.output_P;

        // DebugData.debug1 = pitchAngle;
        // DebugData.debug1 = Motor_Yaw.angle;

        // DebugData.debug1 = pitchSpeed;
        // DebugData.debug2 = Motor_Pitch.angle;
        // DebugData.debug3 = PID_Cloud_PitchAngle.output;
        // DebugData.debug4 = PID_Cloud_PitchSpeed.output;
        // DebugData.debug5 = pitchAngleTarget;
        // DebugData.debug6 = pitchAngleTargetRamp;
        // DebugData.debug7 = pitchRampStart;

        // DebugData.debug1 = ImuData.gx;
        // DebugData.debug2 = ImuData.gy;
        // DebugData.debug3 = ImuData.gz;

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
    float motorSpeed;
    float yawAngle;
    float filter[6] = {0, 0, 0, 0, 0, 0};
    int   filterp   = 0;
    float motorSpeedStable;
    float power       = 0;
    float powerBuffer = 0;

    //大陀螺
    float   swingAngle = 0;
    float   gearRatio  = 290 / 17;
    float   angleSum   = 0;
    int     gearRound  = 0;
    int16_t SwingMode  = 0;
    int16_t swingMode  = 0;

    // 底盘跟随PID
    // float gearRatio        = 1; // 290：17
    float followDeadRegion = 3.0;
    PID_Init(&PID_Follow_Angle, 0.18, 0, 0, 1000, 0);
    PID_Init(&PID_Follow_Speed, 8, 0, 0, 1000, 1000);

    // 麦轮速度PID
    PID_Init(&PID_LFCM, 22, 0, 0, 15000, 7500);
    PID_Init(&PID_LBCM, 22, 0, 0, 15000, 7500);
    PID_Init(&PID_RBCM, 22, 0, 0, 15000, 7500);
    PID_Init(&PID_RFCM, 22, 0, 0, 15000, 7500);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    //底盘运动斜坡函数
    float xRampProgress = 0;
    float xRampStart    = 0;
    float xTargetRamp   = 0;
    float yRampProgress = 0;
    float yRampStart    = 0;
    float yTargetRamp   = 0;

    while (1) {
        //设置底盘模式
        SwingMode = remoteData.switchRight == 3 ? 1 : 0;
        // SwingMode = 0;

        // 设置反馈值
        motorSpeed  = Motor_Yaw.speed * RPM2RPS;                // 电机角速度
        yawAngle    = -1 * Gyroscope_EulerData.yaw;             // 逆时针为正
        power       = Judge.powerHeatData.chassis_power;        // 裁判系统功率
        powerBuffer = Judge.powerHeatData.chassis_power_buffer; // 裁判系统功率缓冲

        // 对电机角速度进行平均值滤波
        // filter[filterp] = motorAngle;
        // filterp         = filterp == 5 ? 0 : filterp + 1;
        // int i;
        // for (i = 0; i < 6; i++) {
        //     motorSpeedStable += filter[i];
        // }
        // motorSpeedStable = motorSpeedStable / 6.0f;

        // 底盘跟随死区
        if (SwingMode) {
            followDeadRegion = 0; // 关闭底盘跟随死区
        } else {
            followDeadRegion = 3; // 开启底盘跟随死区
        }

        //大陀螺
        if (SwingMode) {
            if (!swingMode) {
                gearRound       = 0; //云台圈数清零
                Motor_Yaw.round = 0; // Yaw轴圈数清零
                swingMode       = 1;
            }
            motorAngle = Motor_Yaw.angle;               // 电机角度反馈
            swingAngle += 5000 * interval;              //匀速旋转目标值设置
            gearRound = motorAngle / (360 * gearRatio); //计算云台旋转圈数
        } else {
            if (swingMode) {
                swingMode  = 0;
                swingAngle = 0; //底盘跟随目标角设置
            }
            angleSum   = gearRound * 360 * gearRatio;  //减去完整圈数转过的角度
            motorAngle = (Motor_Yaw.angle - angleSum); //电机角度反馈
            if (ABS(PID_Follow_Angle.error) < followDeadRegion) {
                Motor_Yaw.round = 0;               //圈数清零
                motorAngle      = Motor_Yaw.angle; // 电机角度反馈
            }
            // if (Motor_Yaw.speed == 0) {
            //     Motor_Yaw.round = 0;               //圈数清零
            //     motorAngle      = Motor_Yaw.angle; // 电机角度反馈
            // }
        }

        // 根据运动模式计算PID
        PID_Calculate(&PID_Follow_Angle, swingAngle,
                      motorAngle); // 计算航向角角度PID
        PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output,
                      motorSpeed); // 计算航向角角速度PID

        // 设置底盘总体移动速度
        // if (ControlMode == 1) {
        vx = -remoteData.lx / 660.0f * 4;
        vy = remoteData.ly / 660.0f * 12;
        // } else if (ControlMode == 2) {
        //     xTargetRamp = RAMP(xRampStart, 660, xRampProgress);
        //     if (xRampProgress < 0.5) {
        //         xRampProgress += 0.004f;
        //     } else if (xRampProgress > 0.5 && xRampProgress < 1) {
        //         xRampProgress += 0.002f;
        //     }
        //     yTargetRamp = RAMP(yRampStart, 660, yRampProgress);
        //     if (yRampProgress < 0.5) {
        //         yRampProgress += 0.006f;
        //     } else if (yRampProgress > 0.5 && yRampProgress < 1) {
        //         yRampProgress += 0.004f;
        //     }
        //     MIAO(xRampProgress, 0, 1);
        //     MIAO(yRampProgress, 0, 1);

        //     vx = (keyboardData.A - keyboardData.D) * xTargetRamp / 660.0f * 4;
        //     vy = (keyboardData.W - keyboardData.S) * yTargetRamp / 660.0f * 12;

        //     if (keyboardData.W == 0 && keyboardData.S == 0) {
        //         yRampProgress = 0;
        //         yRampStart    = 0;
        //     }S
        //     if (keyboardData.A == 0 && keyboardData.D == 0) {
        //         xRampProgress = 0;
        //         xRampStart    = 0;
        //     }
        // }

        vw = ABS(PID_Follow_Angle.error) < followDeadRegion ? 0 : (PID_Follow_Speed.output * DPS2RPS);
        // if (ABS(remoteData.rx) > 300 || ABS(mouseData.x) > 41) {
        //     vy = vy / 5.0f;
        //     vx = vx / 2.0f;
        // }

        // 麦轮解算及限速
        targetPower = 80.0 - WANG(40.0 - ChassisData.powerBuffer, 0, 40) / 40.0 * 80.0; // 设置目标功率
        Chassis_Update(&ChassisData, vx, vy, vw);                                       // 更新麦轮转速
        Chassis_Fix(&ChassisData, -motorAngle * 17 / 290);
        // 修正旋转后底盘的前进方向
        Chassis_Calculate_Rotor_Speed(&ChassisData); // 麦轮解算
        Chassis_Limit_Rotor_Speed(&ChassisData,
                                  CHASSIS_ROTOR_SPEED); // 设置转子速度上限 (rad/s)
        Chassis_Limit_Power(&ChassisData, targetPower, power, powerBuffer,
                            interval); // 根据功率限幅
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
                 0x1FF,
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
        // DebugData.debug7 = remoteData.rx;

        DebugData.debug1 = motorAngle;
        DebugData.debug2 = gearRound;
        DebugData.debug3 = ABS(PID_Follow_Angle.error);
        DebugData.debug4 = Motor_Yaw.angle;
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

    int maxShootHeat = Judge.robotState.shooter_heat1_cooling_limit;

    uint8_t shootEnabled      = 0;
    uint8_t shootState        = 0;
    uint8_t shootMode         = 0;
    uint8_t limit_switch      = 0;
    uint8_t last_limit_switch = 0;
    int16_t shootTime         = 0;

    float Stir3510_SpeedTarget;
    float Stir3510_SpeedTargetMax;
    float speedRampProgress = 0;
    float speedRampStart    = 0;
    float frictRampProgress = 0;
    float frictTargetRamp   = 0;

    PID_Init(&PID_LeftFrictSpeed, 35, 0.15, 0, 10000, 5000);
    PID_Init(&PID_RightFrictSpeed, 35, 0.1, 0, 10000, 5000);
    PID_Init(&PID_Stir3510Speed, 320, 0.04, 0, 16000, 10000); // p=110  120  //300

    while (1) {
        //进入射击模式
        shootEnabled = 1;

        //开启摩擦轮
        // if (remoteData.switchRight == 3) {
        // PID_Calculate(&PID_LeftFrictSpeed, -230, Motor_LeftFrict.speed * RPM2RPS / 2);
        // PID_Calculate(&PID_RightFrictSpeed, 230, Motor_RightFrict.speed * RPM2RPS / 2);
        frictTargetRamp = RAMP(0, 460, frictRampProgress); // 360  420
        if (frictRampProgress < 1) {
            frictRampProgress += 0.01f;
        }
        PID_Calculate(&PID_LeftFrictSpeed, frictTargetRamp, Motor_LeftFrict.speed * RPM2RPS);
        PID_Calculate(&PID_RightFrictSpeed, -frictTargetRamp, Motor_RightFrict.speed * RPM2RPS);
        LASER_ON;
        Can_Send(CAN2, 0x200, PID_LeftFrictSpeed.output, PID_RightFrictSpeed.output, 0, 0);

        if (shootEnabled) {

            //射击状态
            // shootState = Judge.powerHeatData.shooter_heat0 < maxShootHeat ? 1 : 0;
            shootState = 1;

            //射击模式
            shootMode = remoteData.switchLeft == 1 ? 1 : 0;
            // shootMode = keyboardData.A == 1 ? 1 : 0;

            //限位开关
            limit_switch = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13);
            if (last_limit_switch != limit_switch) {
                vTaskDelay(10);
                if (shootMode == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)) {
                    limit_switch      = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13);
                    last_limit_switch = limit_switch;
                }
            }

            //目标值设定
            // if (remoteData.ry > 30) { // pitch轴抬起时大拨弹反转
            //     PID_Stir3510Speed.p     = 100;
            //     PID_Stir3510Speed.i     = 0;
            //     Stir3510_SpeedTargetMax = 20;
            // } else {
            if (shootState) {
                if (!limit_switch) {
                    if (shootTime < 1000) {
                        PID_Stir3510Speed.p     = 300; // 250
                        Stir3510_SpeedTargetMax = -60;
                        shootTime++;
                    } else {
                        Stir3510_SpeedTargetMax = -50; //-60
                        // Stir3510_SpeedTargetMax = ABS(Motor_Stir3510.speed * RPM2RPS / 2) <= 5 ? 30 : -60;
                        // if (Stir3510_SpeedTargetMax == 20) PID_Stir3510Speed.p = 100; // 30
                    }
                } else if (limit_switch) {
                    Stir3510_SpeedTargetMax = shootMode == 1 ? -50 : 0; //-60
                    if (shootMode == 1) {
                        PID_Stir3510Speed.p = 320; // 280
                    }
                } else if (!shootState) {
                    Stir3510_SpeedTargetMax = 0;
                }
            }

            //斜坡函数
            Stir3510_SpeedTarget = RAMP(speedRampStart, Stir3510_SpeedTargetMax, speedRampProgress);

            if (speedRampProgress < 1) {
                speedRampProgress += 1;
            }
        } else {
            Stir3510_SpeedTarget = 0;
        }

        PID_Calculate(&PID_Stir3510Speed, Stir3510_SpeedTarget, Motor_Stir3510.speed * RPM2RPS / 2);
        // PID_Calculate(&PID_Stir3510Speed, Stir3510_SpeedTargetMax, Motor_Stir3510.speed * RPM2RPS);

        Can_Send(CAN1, 0x200, 0, 0, 0, PID_Stir3510Speed.output);
        // }

        DebugData.debug1 = PID_Stir3510Speed.output;
        DebugData.debug2 = Motor_Stir3510.speed * RPM2RPS / 2;
        DebugData.debug2 = Motor_Stir3510.speed * RPM2RPS;
        DebugData.debug3 = Stir3510_SpeedTarget;
        DebugData.debug4 = Motor_Stir3510.angle;
        DebugData.debug5 = shootTime;

        // DebugData.debug1 = Motor_LeftFrict.speed * RPM2RPS;
        // DebugData.debug2 = Motor_RightFrict.speed * RPM2RPS;
        // DebugData.debug3 = frictTargetRamp;

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
    // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500,
    // NULL, 6, NULL); xTaskCreate(Task_Debug_Magic_Send,
    // "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6,
    // NULL); xTaskCreate(Task_Debug_Gyroscope_Sampling,
    // "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    // xTaskCreate(Task_Fire, "Task_Fire", 400, NULL, 5, NULL);

    //模式切换任务
    // xTaskCreate(Task_Control, "Task_Control", 400, NULL, 4, NULL);

    // 通讯
    // xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}