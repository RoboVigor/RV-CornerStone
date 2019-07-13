/**
 * @brief 步兵 (无滑环)
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
                Can_Send(CAN2, 0x1FF, 0, 0, 0, 0);
                PWM_Set_Compare(&PWM_Snail1, 0.376 * 1250);
                PWM_Set_Compare(&PWM_Snail2, 0.376 * 1250);
                vTaskDelay(2);
            }
        }
        vTaskDelay(2);
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
    int lastSeq = 0;

    // Pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = -1 * Gyroscope_EulerData.pitch;
    float pitchAngleTargetRamp = 0;

    // 初始化云台PID
    PID_Init(&PID_Cloud_YawAngle, 20, 0, 0, 16000, 0);
    PID_Init(&PID_Cloud_YawSpeed, 1, 0, 0, 4000, 0);
    PID_Init(&PID_Cloud_PitchAngle, 20, 0, 0, 16000, 0);
    PID_Init(&PID_Cloud_PitchSpeed, 1.5, 0, 0, 2000, 0);

    while (1) {
        // 重置目标
        yawAngleTarget   = 0;
        pitchAngleTarget = 0;

        // 设置反馈
        yawAngle     = -1 * Gyroscope_EulerData.yaw;    // 逆时针为正
        yawSpeed     = -1 * ImuData.gz / GYROSCOPE_LSB; // 逆时针为正
        pitchAngle   = -1 * Gyroscope_EulerData.pitch;  // 逆时针为正
        pitchSpeed   = ImuData.gx / GYROSCOPE_LSB;      // 逆时针为正
        chassisAngle = Motor_Pitch.angle - pitchAngle;

        // 遥控器输入角度目标
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl += remoteData.rx / 660.0f * 360 * interval;
        if (ABS(remoteData.ry) > 30) pitchAngleTargetControl += remoteData.ry / 660.0f * 270 * interval;
        MIAO(pitchAngleTargetControl, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;

        // 视觉辅助
        if (remoteData.switchLeft != 3) {
            lastSeq = 0;
            yawAngleTargetControl += yawAngleTargetPs;
            pitchAngleTargetControl += pitchAngleTargetPs;
            yawAngleTargetPs   = 0;
            pitchAngleTargetPs = 0;
        } else if (lastSeq != Ps.autoaimData.seq) {
            lastSeq = Ps.autoaimData.seq;
            yawAngleTargetPs += Ps.autoaimData.yaw_angle_diff;
            pitchAngleTargetPs += Ps.autoaimData.pitch_angle_diff;
        }
        MIAO(pitchAngleTargetPs, GIMBAL_PITCH_MIN - pitchAngleTarget, GIMBAL_PITCH_MAX - pitchAngleTarget);
        yawAngleTarget += yawAngleTargetPs;
        pitchAngleTarget += pitchAngleTargetPs;

        // 上坡补偿
        pitchAngleTargetFixStable = FirstOrderLowPassFilter(-1 * (chassisAngle / 40.0) * (GIMBAL_PITCH_MIN - pitchAngleTarget), &pitchAngleTargetFix, 200, 20);
        pitchAngleTarget += pitchAngleTargetFixStable;

        // 限制云台运动范围
        MIAO(pitchAngleTarget, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);

        // 开机时pitch轴匀速抬起
        pitchAngleTargetRamp = RAMP(pitchRampStart, pitchAngleTarget, pitchRampProgress);
        if (pitchRampProgress < 1) {
            pitchRampProgress += 0.01f;
        }

        // 计算PID
        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, -1 * Gyroscope_EulerData.yaw);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);
        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTargetRamp, pitchAngle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchSpeed);

        // 输出电流
        Can_Send(CAN1, 0x1FF, 20 * PID_Cloud_YawSpeed.output, -25 * PID_Cloud_PitchSpeed.output, 0, 0);
        // Can_Send(CAN1, 0x1FF, remoteData.rx * 10, 0, 0, 0);

        // 调试信息
        // DebugData.debug1 = PID_Cloud_YawSpeed.output;
        // DebugData.debug2 = PID_Cloud_YawAngle.output;
        // DebugData.debug3 = PID_Cloud_YawAngle.target;
        // DebugData.debug4 = PID_Cloud_YawAngle.feedback;
        // DebugData.debug5 = PID_Cloud_PitchAngle.output;
        // DebugData.debug6 = PID_Cloud_PitchAngle.target;
        // DebugData.debug7 = PID_Cloud_PitchAngle.feedback;
        // DebugData.debug8 = chassisAngle;
        DebugData.debug1 = Ps.seq;
        DebugData.debug2 = Ps.autoaimData.yaw_angle_diff;
        DebugData.debug3 = Judge.powerHeatData.chassis_power;
        DebugData.debug4 = Ps.autoaimData.biu_biu_state;

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
    float motorAngle, motorSpeed;
    float lastMotorAngle = Motor_Yaw.angle;
    float filter[6]      = {0, 0, 0, 0, 0, 0};
    int   filterp        = 0;
    float power          = 0;

    // 小陀螺
    float swingAmplitude = 360;
    float swingDuration  = 1000;
    float swingProgress  = 0;
    int   swingDirection = 1;
    float swingStep      = intervalms / swingDuration;
    float swingAngle     = 0;

    // 底盘跟随PID
    float followDeadRegion = 2.0;
    PID_Init(&PID_Follow_Angle, 1, 0, 0, 1000, 100);
    PID_Init(&PID_Follow_Speed, 10, 0, 0, 1000, 0);

    // 麦轮速度PID
    PID_Init(&PID_LFCM, 28, 0, 0, 15000, 7500);
    PID_Init(&PID_LBCM, 28, 0, 0, 15000, 7500);
    PID_Init(&PID_RBCM, 28, 0, 0, 15000, 7500);
    PID_Init(&PID_RFCM, 28, 0, 0, 15000, 7500);

    // 初始化底盘
    Chassis_Init(&ChassisData);

    while (1) {

        // 设置反馈值
        motorAngle = Motor_Yaw.angle;                   // 电机角度
        motorSpeed = Motor_Yaw.speed * RPM2RPS;         // 电机角速度
        power      = Judge.powerHeatData.chassis_power; // 裁判系统功率

        // 小陀螺
        if (remoteData.switchLeft == 2) {
            swingAngle += 360 * interval;
            followDeadRegion = 0; // 关闭底盘跟随死区
        } else {
            swingDirection   = 1;
            swingProgress    = 0;
            swingAngle       = 0;
            followDeadRegion = 2; // 开启底盘跟随死区
            Motor_Yaw.round  = 0; // 圈数清零
        }
        PID_Calculate(&PID_Follow_Angle, swingAngle, motorAngle);
        PID_Calculate(&PID_Follow_Speed, PID_Follow_Angle.output, motorSpeed);

        // 设置底盘总体移动速度
        vx = remoteData.lx / 660.0f * 4;
        vy = -remoteData.ly / 660.0f * 12;
        vw = ABS(PID_Follow_Angle.error) < followDeadRegion ? 0 : (-1 * PID_Follow_Speed.output * DPS2RPS);

        // 麦轮解算及限速
        targetPower = 80.0 - (60.0 - ChassisData.powerBuffer) / 60.0 * 80.0; // 设置目标功率
        Chassis_Update(&ChassisData, vx, vy, vw);                            // 更新麦轮转速
        Chassis_Fix(&ChassisData, motorAngle);                               // 修正旋转后底盘的前进方向
        Chassis_Calculate_Rotor_Speed(&ChassisData);                         // 麦轮解算
        Chassis_Limit_Rotor_Speed(&ChassisData, CHASSIS_ROTOR_SPEED);        // 设置转子速度上限 (rad/s)
        Chassis_Limit_Power(&ChassisData, 80, targetPower, power, interval); // 根据功率限幅

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调
        // Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

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
 * @brief 发射机构 (拨弹轮)
 */

/*
remainToShoot=0,
counter2=0,
最小射击周期 minDuration=20,
任务循环周期 intervalms=5;
while(1){
    remainToShoot+=(counter2+intervalms)/minDuration;
    remainToShoot-=裁判系统读到的射击数量;
    counter2=(counter2+intervalms)%minDuration;
}
*/

void Task_Fire_Stir(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 射击模式
    enum shootMode_e { shootIdle = 0, shootOnce, shootToDeath }; // 停止, 单发, 连发
    enum shootMode_e shootMode = shootIdle;

    // 热量控制
    int   shootNum        = 0;
    int   mayShootNum     = 0;
    int   maxBulletSpeed  = 0;
    float lastBulletSpeed = 0;
    float maxShootHeat    = 0;

    // PID 初始化
    PID_Init(&PID_StirAngle, 4, 0, 0, 4000, 500);   // 拨弹轮角度环
    PID_Init(&PID_StirSpeed, 20, 0, 0, 4000, 1000); // 拨弹轮速度环

    // 开启激光
    LASER_ON;

    while (1) {
        // 输入射击模式
        if (remoteData.switchRight == 1) {
            shootMode = shootIdle;
        } else {
            shootMode = shootToDeath;
        }

        if (remoteData.switchLeft == 3) {
            if (Ps.autoaimData.biu_biu_state) {
                shootMode = shootToDeath;
            } else {
                shootMode = shootIdle;
            }
        }

        // 热量控制
        // if (lastBulletSpeed < Judge.shootData.bullet_speed) {
        //     maxBulletSpeed  = Judge.shootData.bullet_speed;
        //     lastBulletSpeed = Judge.shootData.bullet_speed;
        // }

        // maxShootHeat = 0.8 * Judge.robotState.shooter_heat0_cooling_limit;
        // mayShootNum  = (Judge.robotState.shooter_heat0_cooling_limit - Judge.powerHeatData.shooter_heat0) / maxBulletSpeed;

        // if (lastBulletSpeed != Judge.shootData.bullet_speed) {
        //     shootNum += 1;
        //     lastBulletSpeed = Judge.shootData.bullet_speed;
        // }
        // if (remoteData.switchRight == 3 && remoteData.switchLeft == 3 && Judge.powerHeatData.shooter_heat0 < maxShootHeat) {
        //     shootMode = 1; //连发
        // } else if (remoteData.switchRight == 3 && remoteData.switchLeft == 2 && (shootNum + 1) < mayShootNum) {
        //     shootMode = 2; // n连发
        // } else {
        //     shootMode = 0; //停止
        // }

        // 控制拨弹轮
        if (shootMode == shootIdle) {
            // PWM_Set_Compare(&PWM_Magazine_Servo, 7);
            Can_Send(CAN2, 0x1FF, 0, 0, 0, 0);
        } else if (shootMode == shootToDeath) {
            // 连发
            // PWM_Set_Compare(&PWM_Magazine_Servo, 15);
            PID_Calculate(&PID_StirSpeed, 65, Motor_Stir.speed * RPM2RPS);
            Can_Send(CAN2, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
        } else if (shootMode = shootOnce) {
            PID_Calculate(&PID_StirAngle, 40 * shootNum, Motor_Stir.angle);
            PID_Calculate(&PID_StirSpeed, PID_StirAngle.output, Motor_Stir.speed * RPM2RPS);
            Can_Send(CAN2, 0x1FF, 0, 0, PID_StirSpeed.output, 0);
            shootNum = 0;
        }

        // DebugData.debug1 = Ps.autoaimData.biu_biu_state;
        // DebugData.debug2 = Judge.powerHeatData.shooter_heat0;
        // DebugData.debug3 = Judge.shootData.bullet_speed;
        // DebugData.debug4 = Judge.robotState.shooter_heat0_cooling_limit;
        // DebugData.debug5 = Judge.robotState.shooter_heat0_cooling_rate;
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 发射机构 (摩擦轮)
 * @todo  使用状态机重写
 */
void Task_Fire_Frict(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    float dutyCycleStart  = 0.376; //起始占空比为37.6
    float dutyCycleMiddle = 0.446; //启动需要到44.6
    float dutyCycleEnd    = 0.486; //加速到你想要的占空比

    float dutyCycleRightSnailTarget = 0.376; //目标占空比
    float dutyCycleLeftSnailTarget  = 0.376;

    float dutyCycleRightSnailProgress1 = 0; //存储需要的两个过程（初始到启动，启动到你想要的速度）
    float dutyCycleLeftSnailProgress1  = 0;
    float dutyCycleRightSnailProgress2 = 0;
    float dutyCycleLeftSnailProgress2  = 0;

    int snailRightState = 0; //标志启动完后需要的延时
    int snailLeftState  = 0;

    while (1) {
        /*来自dji开源，两个snail不能同时启动*/
        dutyCycleEnd = 0.526;

        if (dutyCycleRightSnailProgress1 <= 1) { //初始状态
            dutyCycleRightSnailTarget = RAMP(dutyCycleStart, dutyCycleMiddle,
                                             dutyCycleRightSnailProgress1); //斜坡上升
            dutyCycleRightSnailProgress1 += 0.01f;
        } else {
            if (snailRightState == 0) { //初始状态停留100ms
                vTaskDelay(100);
                snailRightState = 1;
            } else {
                if (dutyCycleRightSnailProgress2 <= 1) { //启动状态
                    dutyCycleRightSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd,
                                                     dutyCycleRightSnailProgress2); //斜坡上升
                    dutyCycleRightSnailProgress2 += 0.001f;
                } else {
                    if (dutyCycleLeftSnailProgress1 <= 1) {
                        dutyCycleLeftSnailTarget = RAMP(dutyCycleStart,
                                                        dutyCycleMiddle, //右摩擦轮启动完毕，左摩擦轮进入初始状态
                                                        dutyCycleLeftSnailProgress1);
                        dutyCycleLeftSnailProgress1 += 0.01f;
                    } else {
                        if (snailLeftState == 0) {
                            vTaskDelay(100);
                            snailLeftState = 1;
                        } else {
                            if (dutyCycleLeftSnailProgress2 <= 1) {
                                dutyCycleLeftSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd, dutyCycleLeftSnailProgress2);
                                dutyCycleLeftSnailProgress2 += 0.001f;
                            }
                        }
                    }
                }
            }
        }
        PWM_Set_Compare(&PWM_Snail1, dutyCycleRightSnailTarget * 1250);
        PWM_Set_Compare(&PWM_Snail2, dutyCycleLeftSnailTarget * 1250);
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
        if (KTV_Play(Music_Soul)) break;
        vTaskDelayUntil(&LastWakeTime, 60);
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
    // xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    xTaskCreate(Task_Fire_Stir, "Task_Fire_Stir", 400, NULL, 6, NULL);
    xTaskCreate(Task_Fire_Frict, "Task_Fire_Frict", 400, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}