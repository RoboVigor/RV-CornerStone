/**
 * @brief 无人机代码
 * @version 1.2.0
 * 写在最开始：飞机云台遥控器模式：左2 右2 全部停止
 * 左1 右1 初始状态
 * 左2 右1 启动snail
 * lx 控制拨弹轮
 * rx ry 控制云台
 * 有任何问题请联系qq：740670513
 */

#include "tasks.h"
#include "config.h"
#include "macro.h"
#include "handle.h"

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        // Keyboard_Check(&CTRL, &remoteData, KEY_CTRL);

        // if (remoteData.switchRight == 1) {
        controlMode    = 1; //遥控器模式
        FrictEnabled   = remoteData.switchLeft == 2;
        StirEnabled    = (remoteData.switchLeft == 2) && (remoteData.switchRight == 1);
        PsAimEnabled   = (remoteData.switchLeft == 1) && (remoteData.switchRight == 1);
        PsShootEnabled = 0;
        SafetyMode     = remoteData.switchRight == 2 && remoteData.switchLeft == 2;
        // }
        // else if (remoteData.switchRight != 1) {
        //     controlMode = 2; //键鼠模式
        // }
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

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (SafetyMode) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(2);
    }
    vTaskDelete(NULL);
}

void Task_Snail(void *Parameters) {
    // snail摩擦轮任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟

    float dutyCycleStart  = 0.376; //起始占空比为37.6
    float dutyCycleMiddle = 0.446; //启动需要到44.6
    float dutyCycleEnd    = 0.550; //加速到你想要的占空比   0.550

    float dutyCycleRightSnailTarget = 0.376; //目标占空比
    float dutyCycleLeftSnailTarget  = 0.376;

    float dutyCycleRightSnailProgress1 = 0; //存储需要的两个过程（初始到启动，启动到你想要的速度）
    float dutyCycleLeftSnailProgress1  = 0;
    float dutyCycleRightSnailProgress2 = 0;
    float dutyCycleLeftSnailProgress2  = 0;

    int snailRightState = 0; //标志启动完后需要的延时
    int snailLeftState  = 0;

    int snailState = 0; //标志启动完后需要的延时

    int lastMouseDataRight = 0;

    snailStart = 0;

    /*来自dji开源，两个snail不能同时启动*/

    while (1) {

        GPIO_SetBits(GPIOG, GPIO_Pin_13); //激光
                                          //启动摩擦轮

        if (!FrictEnabled) {
            dutyCycleRightSnailTarget    = 0.376;
            dutyCycleLeftSnailTarget     = 0.376;
            dutyCycleRightSnailProgress1 = 0;
            dutyCycleRightSnailProgress2 = 0;
            dutyCycleLeftSnailProgress1  = 0;
            dutyCycleLeftSnailProgress2  = 0;
        } else {

            if (dutyCycleRightSnailProgress1 <= 1) { //初始状态
                dutyCycleRightSnailTarget = RAMP(dutyCycleStart, dutyCycleMiddle,
                                                 dutyCycleRightSnailProgress1); //斜坡上升
                dutyCycleRightSnailProgress1 += 0.05f;
            } else {
                if (dutyCycleLeftSnailProgress1 <= 1) {
                    dutyCycleLeftSnailTarget = RAMP(dutyCycleStart,
                                                    dutyCycleMiddle, //右摩擦轮启动完毕，左摩擦轮进入初始状态
                                                    dutyCycleLeftSnailProgress1);
                    dutyCycleLeftSnailProgress1 += 0.05f;
                } else if (snailLeftState == 0) {
                    vTaskDelay(100);
                    snailLeftState = 1;
                } else if (dutyCycleLeftSnailProgress2 <= 1) {
                    dutyCycleLeftSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd, dutyCycleLeftSnailProgress2);
                    dutyCycleLeftSnailProgress2 += 0.005f;
                }
                if (snailRightState == 0) { //初始状态停留100ms
                    vTaskDelay(100);
                    snailRightState = 1;
                } else if (dutyCycleRightSnailProgress2 <= 1) {
                    dutyCycleRightSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd,
                                                     dutyCycleRightSnailProgress2); //斜坡上升
                    dutyCycleRightSnailProgress2 += 0.005f;
                }
            }
        }

        PWM_Set_Compare(&PWM_Snail1, dutyCycleRightSnailTarget * 1250);
        PWM_Set_Compare(&PWM_Snail2, dutyCycleLeftSnailTarget * 1250);

        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

// 云台任务关于角度使用的是陀螺仪
void Task_Gimbal(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    // 反馈值
    float yawAngle       = 0;
    float yawSpeed       = 0;
    float pitchAngle     = 0;
    float pitchSpeed     = 0;
    float lastPitchAngle = 0;
    float lastPitchSpeed = 0;
    float rollAngle      = 0;
    float rollSpeed      = 0;
    float lastRollAngle  = 0;
    float lastRollSpeed  = 0;
    float x              = 0;

    // 目标值
    // float yawAngleTarget          = Gyroscope_EulerData.yaw; // 初始目标角度初始化
    float yawAngleTarget          = -1 * Motor_Yaw.angle;   // 初始目标角度初始化
    float pitchAngleTarget        = -1 * Motor_Pitch.angle; // 目标Pitch
    float yawAngleTargetControl   = 0;                      // 遥控器输入
    float pitchAngleTargetControl = 0;                      // 遥控器输入
    float yawAngleTargetPs        = 0;                      // 视觉辅助
    float pitchAngleTargetPs      = 0;                      // 视觉辅助
    float rollAngleTarget         = 0;

    //初始化云台PID
    //加d抑制抖动 但是在起飞时由于高速震动会导致云台抖动
    PID_Init(&PID_Cloud_YawAngle, 20, 0, 0, 20000, 0);     // 120     20
    PID_Init(&PID_Cloud_YawSpeed, 170, 0, 0, 20000, 0);    // 30      150
    PID_Init(&PID_Cloud_PitchAngle, 50, 0, 0, 20000, 0);   // 100     20
    PID_Init(&PID_Cloud_PitchSpeed, 50, 0, 0, 20000, 0);   // 30      100
    PID_Init(&PID_Cloud_RollAngle, 20, 0, 0, 15000, 7500); // 70  150
    PID_Init(&PID_Cloud_RollSpeed, 100, 0, 0, 20000, 0);   // 50  35

    // pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = -1 * Motor_Pitch.angle;
    float pitchAngleTargetRamp = 0;

    // yaw轴斜坡参数
    float yawRampProgress    = 0;
    float yawRampStart       = -1 * Motor_Yaw.angle;
    float yawAngleTargetRamp = 0;

    // roll轴斜坡参数
    float rollRampProgress    = 0;
    float rollRampStart       = Gyroscope_EulerData.roll;
    float rollAngleTargetRamp = 0;

    int lastSeq = 0;

    while (1) {
        // 重置目标
        yawAngleTarget   = 0;
        pitchAngleTarget = 0;

        // 设置反馈
        // yawAngle = Gyroscope_EulerData.yaw;
        // yawSpeed = ImuData.gz / GYROSCOPE_LSB;
        yawAngle = -1 * Motor_Yaw.angle;
        yawSpeed = -1 * Motor_Yaw.speed;
        // pitchAngle = -1 * Gyroscope_EulerData.pitch; // 逆时针为正
        // pitchSpeed = ImuData.gx / GYROSCOPE_LSB;     // 逆时针为正
        pitchAngle = -1 * Motor_Pitch.angle;
        pitchSpeed = -1 * Motor_Pitch.speed;
        rollAngle  = Gyroscope_EulerData.roll;
        rollSpeed  = ImuData.gy / GYROSCOPE_LSB;

        // 遥控器输入角度目标
        if (ABS(remoteData.rx) > 30) yawAngleTargetControl += -remoteData.rx / 660.0f * 20 * 0.01;
        if (ABS(remoteData.ry) > 30) pitchAngleTargetControl += -remoteData.ry / 660.0f * 30 * 0.01;
        yawAngleTargetControl += mouseData.x * 0.5 * 0.005; // 0.005
        pitchAngleTargetControl += mouseData.y * 0.005;

        MIAO(yawAngleTargetControl, GIMBAL_YAW_MIN, GIMBAL_YAW_MAX);
        MIAO(pitchAngleTargetControl, GIMBAL_PITCH_MIN, GIMBAL_PITCH_MAX);
        yawAngleTarget += yawAngleTargetControl;
        pitchAngleTarget += pitchAngleTargetControl;

        // // 视觉辅助
        // if (!PsAimEnabled) {
        //     lastSeq = Node_Host.seq;
        //     yawAngleTargetControl += yawAngleTargetPs;
        //     pitchAngleTargetControl += pitchAngleTargetPs;
        //     yawAngleTarget += yawAngleTargetPs;
        //     pitchAngleTarget += pitchAngleTargetPs;
        //     yawAngleTargetPs   = 0;
        //     pitchAngleTargetPs = 0;
        // } else {
        //     if (lastSeq != Node_Host.seq) {
        //         lastSeq = Node_Host.seq;
        //         yawAngleTargetPs += ProtocolData.host.autoaimData.yaw_angle_diff;
        //         pitchAngleTargetPs -= ProtocolData.host.autoaimData.pitch_angle_diff;
        //     }
        // }
        // MIAO(yawAngleTargetPs, GIMBAL_YAW_MIN - yawAngleTarget, GIMBAL_YAW_MAX - yawAngleTarget);
        // MIAO(pitchAngleTargetPs, GIMBAL_PITCH_MIN - pitchAngleTarget, GIMBAL_PITCH_MAX - pitchAngleTarget);
        // yawAngleTarget += yawAngleTargetPs;
        // pitchAngleTarget += pitchAngleTargetPs;

        //开机时pitch轴匀速抬起
        pitchAngleTargetRamp = RAMP(pitchRampStart, pitchAngleTarget, pitchRampProgress);
        if (pitchRampProgress < 1) {
            pitchRampProgress += 0.005f;
        }

        //开机时yaw轴匀速抬起
        yawAngleTargetRamp = RAMP(yawRampStart, yawAngleTarget, yawRampProgress);
        if (yawRampProgress < 1) {
            yawRampProgress += 0.005f;
        }

        //开机时roll轴匀速抬起
        rollAngleTargetRamp = RAMP(rollRampStart, rollAngleTarget, rollRampProgress);
        if (rollRampProgress < 1) {
            rollRampProgress += 0.005f;
        }

        // pitch滤波
        // pitchAngle     = 0.7 * lastPitchAngle + 0.3 * pitchAngle;
        // lastPitchAngle = pitchAngle;

        // roll滤波
        if (ABS((rollAngle - lastRollAngle) * 1000) < 100) {
            rollAngle = lastRollAngle;
        } else {
            rollAngle = 0.6 * lastRollAngle + 0.4 * rollAngle;
        }
        lastRollAngle = rollAngle;

        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTargetRamp, yawAngle);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);

        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTargetRamp, pitchAngle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchSpeed);

        PID_Calculate(&PID_Cloud_RollAngle, rollAngleTargetRamp, rollAngle);
        PID_Calculate(&PID_Cloud_RollSpeed, PID_Cloud_RollAngle.output, rollSpeed);

        // 输出电流
        // Can_Send(CAN1, 0x1FF, -PID_Cloud_YawSpeed.output, -PID_Cloud_RollSpeed.output, -1 * PID_Cloud_PitchSpeed.output, 0);
        Motor_Yaw.input   = -PID_Cloud_YawSpeed.output;
        Motor_Pitch.input = -PID_Cloud_RollSpeed.output;
        Motor_Roll.input  = -PID_Cloud_PitchSpeed.output;
        Motor_Stir.input  = 0;

        // DebugData.debug1 = PID_Cloud_YawSpeed.p;
        // DebugData.debug2 = Motor_Yaw.position;
        // DebugData.debug3 = Motor_Yaw.positionBias;
        // DebugData.debug4 = pitchAngleTarget;

        vTaskDelayUntil(&LastWakeTime, 5);
    }
}

//射击看不懂就重写一下吧 有一部分是代码限制拨弹盘卡死的代码
void Task_Fire(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      rpm2rps      = 3.14 / 60;           // 转子的转速(round/min)换算成角速度(rad/s)
    float      r            = 0.0595;
    // uint8_t    startCounter = 0;                   // 启动模式计数器

    LASER_OFF;

    // 标志位
    uint8_t frictState = 0;
    uint8_t stirState  = 0;
    uint8_t stirFlag   = 0;

    int turnNumber = 1;
    int lastSwitch = 2; //
    // 常量
    // float frictSpeed = -24 / 0.0595 * 2 * 60 / 2 / 3.14;
    // 摩擦轮线速度(mps)转转速(rpm)
    float frictSpeed = 336; // 336 rad/s
    float stirSpeed  = 36 * 36;
    float stirAmpre  = 0;

    // PID 初始化
    // PID_Init(&PID_StirSpeed, 5, 0.005, 0, 6000, 3000); // 1.8
    PID_Init(&PID_StirSpeed, 3, 0.01, 0, 10000, 3000); // 1.8

    int      stirstate     = 0; //播弹轮开启为1 关闭为0
    uint32_t stircount1    = 0; //堵转计数器 保证反转
    uint32_t stircount2    = 0;
    int      stopstate     = 0; //堵转模式 1为堵转 0为正常
    float    currentTarget = 0;
    int      counter       = 0;

    while (1) {
        //拨弹轮 PID 控制
        // if (remoteData.switchRight == 1) {
        //
        // } else if (remoteData.switchRight == 3) {
        //
        // }

        if (StirEnabled) {
            PWM_Set_Compare(&PWM_Servo, 23);
            if (counter < 100) {
                counter++;
                currentTarget = 0;
            } else {
                PID_Calculate(&PID_StirSpeed, 4000, Motor_Stir.speed);
                currentTarget = PID_StirSpeed.output;
            }
        } else {
            PWM_Set_Compare(&PWM_Servo, 19);
            currentTarget = 0;
            counter       = 0;
        }

        Can_Send(CAN1, 0x200, 0, 0, currentTarget, 0);

        DebugData.debug5 = counter;
        // DebugData.debug6 = Motor_Stir.speed;
        // DebugData.debug7 = Motor_Stir.angle;
        // DebugData.debug8 = ImuData.gz;
        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

// void Task_OLED(void *Parameters) {
//     uint16_t   JoystickValue = -1;
//     TickType_t LastWakeTime  = xTaskGetTickCount();
//     oled_init();
//     while (1) {
//         JoystickValue = ADC_GetConversionValue(ADC1);
//         oled_clear(Pen_Clear);
//         oled_menu(JoystickValue);
//         oled_refresh_gram();
//         vTaskDelayUntil(&LastWakeTime, 125);
//     }
//     vTaskDelete(NULL);
// }

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