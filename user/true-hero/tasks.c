/**
 * @brief 真*英雄
 * @version 1.2.0
 */
#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {

        if (remoteData.switchLeft == 1 || remoteData.switchLeft == 3) {
            ControlMode  = 1;                           //遥控器模式
            SafetyMode   = RIGHT_SWITCH_BOTTOM;         //安全模式
            ShootEnabled = !LEFT_SWITCH_BOTTOM;         //射击模式
            SwingMode    = RIGHT_SWITCH_MIDDLE ? 1 : 0; //大陀螺
            ShootMode    = LEFT_SWITCH_TOP ? 1 : 0;     // 发射
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
    float pitchRampProgress    = 0;
    float pitchRampStart       = Motor_Pitch.angle;
    float pitchAngleTargetRamp = 0;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 20, 0, 0, 3000, 0);
    PID_Init(&PID_Cloud_YawSpeed, 40, 0, 0, 16000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 15, 0, 0, 1000, 0);   // 20
    PID_Init(&PID_Cloud_PitchSpeed, 150, 0, 0, 16000, 0); // 150
    // PID_Cloud_PitchAngle.p = CHOOSE(15, 20, 25);

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
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f * 1.5;
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
        // if (!PsEnabled) {
        //     lastSeq = Ps.autoaimData.seq;
        // } else if (lastSeq != Ps.autoaimData.seq) {
        //     lastSeq = Ps.autoaimData.seq;
        //     yawAngleTargetPs += Ps.autoaimData.yaw_angle_diff;
        //     pitchAngleTargetPs -= Ps.autoaimData.pitch_angle_diff;
        // }
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
            pitchRampProgress += 0.005f;
        }

        // 计算PID
        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);

        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTargetRamp, -Motor_Pitch.angle); // pitch轴电机
        // PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTargetRamp, pitchAngle);//陀螺仪
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchSpeed); //正输出向上

        // 输出电流值到电调
        Motor_Yaw.input   = PID_Cloud_YawSpeed.output;
        Motor_Pitch.input = PID_Cloud_PitchSpeed.output;

        // 调试信息
        // DebugData.debug1 = yawAngle;
        // DebugData.debug2 = yawSpeed;
        // DebugData.debug3 = PID_Cloud_YawAngle.output;
        // DebugData.debug4 = PID_Cloud_YawSpeed.output;
        // DebugData.debug5 = yawAngleTarget;
        // DebugData.debug6 = PID_Cloud_YawAngle.error;
        // DebugData.debug7 = PID_Cloud_YawAngle.output_P;

        // DebugData.debug1 = pitchAngle;
        // DebugData.debug1 = -Motor_Pitch.angle;

        // DebugData.debug1 = pitchSpeed;
        // DebugData.debug2 = -Motor_Pitch.angle;
        // DebugData.debug3 = PID_Cloud_PitchAngle.output;
        // DebugData.debug4 = PID_Cloud_PitchSpeed.output;
        // DebugData.debug5 = pitchAngleTarget;
        // DebugData.debug6 = pitchAngleTargetRamp;
        // DebugData.debug7 = PID_Cloud_PitchAngle.p;
        // DebugData.debug7 = pitchRampStart;

        // DebugData.debug1 = ImuData.gx;
        // DebugData.debug2 = ImuData.gy;
        // DebugData.debug3 = ImuData.gz;

        // DebugData.debug1 = Motor_Yaw.angle;
        // DebugData.debug2 = yawAngle;

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
    int16_t motorMode  = 1;
    int16_t swingMode  = 0;

    // 底盘跟随PID
    float followDeadRegion = 3.0;
    PID_Init(&PID_Follow_Angle, 0.1, 0, 0, 1000, 0);
    PID_Init(&PID_Follow_Speed, 10, 0, 0, 1000, 1000);
    // PID_Follow_Speed.p = CHOOSE(8, 10, 12);

    // 麦轮速度PID
    PID_Init(&PID_LFCM, 25, 0, 0, 15000, 7500);
    PID_Init(&PID_LBCM, 25, 0, 0, 15000, 7500);
    PID_Init(&PID_RBCM, 25, 0, 0, 15000, 7500);
    PID_Init(&PID_RFCM, 25, 0, 0, 15000, 7500);

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

        // 设置反馈值
        motorSpeed  = Motor_Yaw.speed * RPM2RPS;                       // 电机角速度
        yawAngle    = -1 * Gyroscope_EulerData.yaw;                    // 逆时针为正
        power       = ProtocolData.powerHeatData.chassis_power;        // 裁判系统功率
        powerBuffer = ProtocolData.powerHeatData.chassis_power_buffer; // 裁判系统功率缓冲

        // 底盘跟随死区
        if (SwingMode) {
            followDeadRegion = 0; // 关闭底盘跟随死区
        } else {
            followDeadRegion = 1; // 开启底盘跟随死区
        }

        //大陀螺
        if (SwingMode) {
            if (!swingMode) {
                Motor_Yaw.round        = 0;                  // Yaw轴圈数清零
                Motor_Yaw.positionBias = Motor_Yaw.position; //角度清零
                swingMode              = 1;
                motorMode              = 0;
            }
            motorAngle = Motor_Yaw.angle;               // 电机角度反馈
            swingAngle += 5000 * interval;              //匀速旋转目标值设置
            gearRound = motorAngle / (360 * gearRatio); //计算云台旋转圈数
        } else {
            if (swingMode) {
                swingMode  = 0;
                swingAngle = 0;
            }
            angleSum   = gearRound * 6141.9;
            motorAngle = (Motor_Yaw.angle - angleSum); //电机角度反馈
            //未回正前
            if (!motorMode) {
                if (ABS(motorAngle) < 1) {
                    Motor_Yaw.round        = 0;                  //圈数清零
                    Motor_Yaw.positionBias = Motor_Yaw.position; //角度清零
                    gearRound              = 0;                  //云台圈数清零
                    motorAngle             = Motor_Yaw.angle;    //更新反馈
                    motorMode              = 1;                  //只清零一次
                }
            }
        }

        // 根据运动模式计算PID
        PID_Calculate(&PID_Follow_Angle, swingAngle,
                      motorAngle); // 计算航向角角度PID
        PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output,
                      motorSpeed); // 计算航向角角速度PID

        // 设置底盘总体移动速度
        if (ControlMode == 1) {
            vx = -remoteData.lx / 660.0f * 8;
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
            S if (keyboardData.A == 0 && keyboardData.D == 0) {
                xRampProgress = 0;
                xRampStart    = 0;
            }
        }

        vw = ABS(PID_Follow_Angle.error) < followDeadRegion ? 0 : (PID_Follow_Speed.output * DPS2RPS);
        if (ABS(remoteData.rx) > 300 || ABS(mouseData.x) > 41) {
            vy = vy / 5.0f;
            vx = vx / 2.0f;
        }

        // 麦轮解算及限速
        targetPower = 80.0 - WANG(40.0 - ChassisData.powerBuffer, 0, 40) / 40.0 * 80.0; // 设置目标功率
        Chassis_Update(&ChassisData, vx, vy, vw);                                       // 更新麦轮转速
        Chassis_Fix(&ChassisData, -motorAngle / gearRatio);
        // 修正旋转后底盘的前进方向
        Chassis_Calculate_Rotor_Speed(&ChassisData); // 麦轮解算
        Chassis_Limit_Rotor_Speed(&ChassisData,
                                  CHASSIS_ROTOR_SPEED); // 设置转子速度上限 (rad/s)
        // Chassis_Limit_Power(&ChassisData, targetPower, power, powerBuffer,
        // interval); // 根据功率限幅
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
        Motor_LF.input = PID_LFCM.output;
        Motor_LB.input = PID_LBCM.output;
        Motor_RB.input = PID_RBCM.output;
        Motor_RF.input = PID_RFCM.output;

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = ABS(PID_Follow_Angle.error);
        // DebugData.debug2 = -1 * PID_Follow_Speed.output   * DPS2RPS * 5 * 1000;
        // DebugData.debug3 = motorSpeedStable;
        // DebugData.debug4 = PID_LFCM.output;
        // DebugData.debug5 = followDeadRegion;
        // DebugData.debug7 = remoteData.rx;

        // DebugData.debug1 = motorAngle;
        // DebugData.debug2 = swingAngle;
        // DebugData.debug3 = Motor_Yaw.position;
        // DebugData.debug4 = Motor_Yaw.round;
        // DebugData.debug5 = gearRound;
        // DebugData.debug6 = ABS(PID_Follow_Angle.error);
        // DebugData.debug7 = Motor_Yaw.angle;
        // DebugData.debug8 = Motor_Yaw.positionBias;

        // DebugData.debug1 = ChassisData.rotorSpeed[0];
        // DebugData.debug2 = Motor_LF.speed * RPM2RPS;
        // DebugData.debug3 = ChassisData.rotorSpeed[1];
        // DebugData.debug4 = Motor_LB.speed * RPM2RPS;
        // DebugData.debug5 = ChassisData.rotorSpeed[2];
        // DebugData.debug6 = Motor_RB.speed * RPM2RPS;
        // DebugData.debug7 = vx;
        // DebugData.debug8 = vy;

        // DebugData.debug1 = motorAngle;
        // DebugData.debug2 = gearRound;
        // DebugData.debug3 = ABS(PID_Follow_Angle.error);
        // DebugData.debug4 = Motor_Yaw.angle;
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

    int     maxShootHeat      = ProtocolData.robotState.shooter_heat1_cooling_limit;
    uint8_t ShootState        = 0;
    uint8_t limit_switch      = 0;
    uint8_t last_limit_switch = 0;
    int16_t shootTime         = 0;

    float Stir3510_SpeedTarget;
    float speedRampProgress = 0;
    float speedRampStart    = 0;
    float frictRampProgress = 0;
    float frictTargetRamp   = 0;

    PID_Init(&PID_LeftFrictSpeed, 35, 0.15, 0, 10000, 5000);
    PID_Init(&PID_RightFrictSpeed, 35, 0.1, 0, 10000, 5000);
    PID_Init(&PID_Stir3510Speed, 85, 0, 0, 16000, 10000); // p=110  120  //300  320
    // PID_Stir3510Speed.p = CHOOSE(50, 55, 60);

    while (1) {
        ShootEnabled = 1;

        if (ShootEnabled) {
            //开启摩擦轮
            frictTargetRamp = RAMP(0, 460, frictRampProgress); // 360  420  460
            if (frictRampProgress < 1) {
                frictRampProgress += 0.01f;
            }
            PID_Calculate(&PID_LeftFrictSpeed, -frictTargetRamp, Motor_LeftFrict.speed * RPM2RPS);
            PID_Calculate(&PID_RightFrictSpeed, frictTargetRamp, Motor_RightFrict.speed * RPM2RPS);
            //开启激光
            LASER_ON;

            //射击状态
            // ShootState = Judge.powerHeatData.shooter_heat0 < maxShootHeat ? 1 : 0;
            ShootState = 1;

            //限位开关
            limit_switch = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13);
            if (last_limit_switch != limit_switch) {
                vTaskDelay(10);
                if (ShootMode == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)) {
                    limit_switch      = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13);
                    last_limit_switch = limit_switch;
                }
            }

            //目标值设定
            if (ShootState) {
                if (!limit_switch) {
                    if (shootTime < 1000) {
                        PID_Stir3510Speed.p  = 90; // 250
                        Stir3510_SpeedTarget = -65;
                        shootTime++;
                    } else {
                        Stir3510_SpeedTarget = -65; //-60
                    }
                } else if (limit_switch) {
                    Stir3510_SpeedTarget = ShootMode == 1 ? -70 : 0; //-60
                    if (ShootMode == 1) {
                        PID_Stir3510Speed.p = 120; // 280
                    }
                } else if (!ShootState) {
                    Stir3510_SpeedTarget = 0;
                }
            }

        } else {
            Stir3510_SpeedTarget = 0;
        }

        PID_Calculate(&PID_Stir3510Speed, Stir3510_SpeedTarget, Motor_Stir3510.speed * RPM2RPS / 2);

        // 输出电流值到电调
        Motor_LeftFrict.input  = PID_LeftFrictSpeed.output;
        Motor_RightFrict.input = PID_RightFrictSpeed.output;
        Motor_Stir3510.input   = PID_Stir3510Speed.output;

        DebugData.debug1 = PID_Stir3510Speed.output;
        DebugData.debug2 = Motor_Stir3510.speed * RPM2RPS / 2;
        // DebugData.debug2 = Motor_Stir3510.speed * RPM2RPS;
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