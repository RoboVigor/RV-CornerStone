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
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2 && remoteData.switchLeft == 2) {
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
    float dutyCycleEnd    = 0.550; //加速到你想要的占空比

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
        //遥控器模式 暂时注释
        // if (controlMode == 1) {
        //     if (remoteData.switchLeft == 1) {
        //         snailStart=0;
        //     }else{
        //         snailStart=1;
        //     }
        // }else{

        //     if (mouseData.pressRight == 1) {
        //         snailStart=1;
        //         lastMouseDataRight = mouseData.pressRight;
        //     }else if(mouseData.pressRight == 0){
        //         snailStart=0;
        //         lastMouseDataRight = mouseData.pressRight;
        //     }else{
        // 			lastMouseDataRight = mouseData.pressRight;
        // 		}
        // }
        //启动摩擦轮
        if (remoteData.switchLeft == 1) {
            snailStart = 0;
        } else {
            snailStart = 1;
        }

        if (snailStart == 0) {

            dutyCycleRightSnailTarget    = 0.376;
            dutyCycleLeftSnailTarget     = 0.376;
            dutyCycleRightSnailProgress1 = 0;
            dutyCycleRightSnailProgress2 = 0;
            dutyCycleLeftSnailProgress1  = 0;
            dutyCycleLeftSnailProgress2  = 0;
        }

        else if (snailStart == 1) {

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
                } else {
                    if (snailLeftState == 0) {
                        vTaskDelay(100);
                        snailLeftState = 1;
                    } else {
                        if (dutyCycleLeftSnailProgress2 <= 1) {
                            dutyCycleLeftSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd, dutyCycleLeftSnailProgress2);
                            dutyCycleLeftSnailProgress2 += 0.005f;
                        }
                    }
                }
                if (snailRightState == 0) { //初始状态停留100ms
                    vTaskDelay(100);
                    snailRightState = 1;
                } else {
                    if (dutyCycleRightSnailProgress2 <= 1) { //启动状态
                        dutyCycleRightSnailTarget = RAMP(dutyCycleMiddle, dutyCycleEnd,
                                                         dutyCycleRightSnailProgress2); //斜坡上升
                        dutyCycleRightSnailProgress2 += 0.005f;
                    }
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
    float rollAngle      = 0;
    float rollSpeed      = 0;
    float lastRollAngle  = 0;
    float x              = 0;

    // 目标值
    //（陀螺仪控制云台）初始目标角度初始化
    float yawAngleTarget   = Gyroscope_EulerData.yaw;
    float pitchAngleTarget = 0;

    //（编码器控制云台）初始目标角度初始化
    // float yawAngleTarget = -Motor_Yaw.angle;
    // float pitchAngleTarget = -1 * Motor_Pitch.angle;

    //初始化云台PID
    //加d抑制抖动 但是在起飞时由于高速震动会导致云台抖动
    PID_Init(&PID_Cloud_YawAngle, 20, 0, 0, 20000, 0);     // 120   15    10
    PID_Init(&PID_Cloud_YawSpeed, 150, 0, 0, 20000, 0);    // 30    1     120
    PID_Init(&PID_Cloud_PitchAngle, 60, 0, 0, 20000, 0);   // 100   2     20
    PID_Init(&PID_Cloud_PitchSpeed, 18, 0, 0, 20000, 0);   // 30  1       100
    PID_Init(&PID_Cloud_RollAngle, 20, 0, 0, 15000, 7500); // 70  150
    PID_Init(&PID_Cloud_RollSpeed, 100, 0, 0, 20000, 0);   // 50  35

    // pitch轴斜坡参数
    float pitchRampProgress    = 0;
    float pitchRampStart       = -1 * Gyroscope_EulerData.pitch;
    float pitchAngleTargetRamp = 0;

    while (1) {
        // 设置反馈
        // 陀螺仪模式
        yawAngle   = Gyroscope_EulerData.yaw;
        yawSpeed   = ImuData.gz / GYROSCOPE_LSB;
        pitchAngle = -1 * Gyroscope_EulerData.pitch; // 逆时针为正
        pitchSpeed = ImuData.gx / GYROSCOPE_LSB;     // 逆时针为正

        //编码器模式
        // yawAngle = -Motor_Yaw.angle;
        // yawSpeed = -Motor_Yaw.speed;
        // pitchAngle = -1 * Motor_Pitch.angle;
        // pitchSpeed = -Motor_Pitch.speed;

        rollAngle = Gyroscope_EulerData.roll;

        rollSpeed = ImuData.gy / GYROSCOPE_LSB;

        //遥控器模式 暂时注释
        // if (controlMode == 1) {
        //     if (ABS(remoteData.rx) > 30) yawAngleTarget += remoteData.rx / 660.0f
        //     * 90 * 0.01; if (ABS(remoteData.ry) > 30) pitchAngleTarget +=
        //     remoteData.ry / 660.0f * 40 * 0.01;
        // } else if (controlMode == 2) {
        //     yawAngleTarget += mouseData.x * 0.01;
        //     pitchAngleTarget += (-mouseData.y) * 0.005;
        // }

        if (ABS(remoteData.rx) > 30) yawAngleTarget += -remoteData.rx / 660.0f * 20 * 0.01;
        if (ABS(remoteData.ry) > 30) pitchAngleTarget += -remoteData.ry / 660.0f * 30 * 0.01;

        PID_Calculate(&PID_Cloud_YawAngle, yawAngleTarget, yawAngle);
        PID_Calculate(&PID_Cloud_YawSpeed, PID_Cloud_YawAngle.output, yawSpeed);

        MIAO(pitchAngleTarget, -22, 40);
        //开机时pitch轴匀速抬起
        pitchAngleTargetRamp = RAMP(pitchRampStart, pitchAngleTarget, pitchRampProgress);
        if (pitchRampProgress < 1) {
            pitchRampProgress += 0.005f;
        }
        // pitch滤波
        lastPitchAngle = pitchAngle;
        pitchAngle     = 0.7 * lastPitchAngle + 0.3 * pitchAngle;

        PID_Calculate(&PID_Cloud_PitchAngle, pitchAngleTargetRamp, pitchAngle);
        PID_Calculate(&PID_Cloud_PitchSpeed, PID_Cloud_PitchAngle.output, pitchSpeed);

        // roll滤波
        if (ABS((rollAngle - lastRollAngle) * 1000) < 100) {
            rollAngle = lastRollAngle;
        } else {
            rollAngle = 0.6 * lastRollAngle + 0.4 * rollAngle;
        }
        lastRollAngle = rollAngle;
        PID_Calculate(&PID_Cloud_RollAngle, 0, rollAngle);
        PID_Calculate(&PID_Cloud_RollSpeed, PID_Cloud_RollAngle.output, rollSpeed);

        // 输出电流
        Can_Send(CAN1, 0x1FF, -PID_Cloud_YawSpeed.output, -PID_Cloud_RollSpeed.output, -PID_Cloud_PitchSpeed.output, 0);

        // Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);

        DebugData.debug1 = Motor_Yaw.position;
        // DebugData.debug2 = yawAngle * 1000;
        DebugData.debug3 = pitchAngleTarget;
        DebugData.debug4 = pitchAngle * 1000;

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

    while (1) {
        // if (controlMode == 1) {
        //     if (remoteData.lx < -60 && snailStart==1) {
        //         stirstate = 1; //正转
        //     }
        //     //  else if (remoteData.lx > 60) {
        //     //     stirstate = -1;
        //     // }
        //     else {
        //         stirstate = 0;
        //     }
        // } else {
        //     if (mouseData.pressLeft == 1 && snailStart == 1) {
        //         stirstate = 1;
        //     } else if (mouseData.pressLeft == 0) {
        //         stirstate = 0;
        //     }else if (mouseData.pressLeft == 1&&snailStart == 0) {
        //         stirstate = 0;
        //         }
        // }
        if (remoteData.lx < -60) {
            stirstate = 1; //正转
        }
        //  else if (remoteData.lx > 60) {
        //     stirstate = -1;
        // }
        else {
            stirstate = 0;
        }

        //拨弹轮 PID 控制

        if (stirstate == 1) {
            // if (stopstate == 0) {
            //     PID_Calculate(&PID_StirSpeed, -700, Motor_Stir.speed * RPM2RPS);
            //     currentTarget = PID_StirSpeed.output;
            // }
            // if (stirstate == 1 && ABS(Motor_Stir.speed * RPM2RPS) < 10 &&
            // stircount1 < 30) {
            //     stircount1 += 1;
            // } else if (stirstate == 1 && currentTarget < 200) {
            //     stircount1 = 0;
            // }
            // if (stircount1 == 30) {
            //     stopstate = 1;
            // } else {
            //     stopstate = 0;
            // }
            // if (stopstate == 1 && stircount2 < 50) {
            //     currentTarget = 2000;
            //     stircount2 += 1;
            // } else if (stircount2 == 50) {
            //     currentTarget = 0;
            //     stircount1    = 0;
            //     stircount2    = 0;
            //     // stircount2 += 1;
            // }
            //  else if (stircount2 == 400) {
            //     stircount1 = 0;
            //     stircount2 = 0;
            // }

            PID_Calculate(&PID_StirSpeed, 4000, Motor_Stir.speed);
            currentTarget = PID_StirSpeed.output;

        } else if (stirstate == 0) {
            currentTarget = 0;
        }
        // else if (stirstate == -1) {
        //     currentTarget = 2000;
        // }

        Can_Send(CAN1, 0x200, 0, 0, currentTarget, 0);

        DebugData.debug5 = currentTarget;
        DebugData.debug6 = Motor_Stir.speed;
        DebugData.debug7 = Motor_Stir.angle;
        DebugData.debug8 = ImuData.gz;
        vTaskDelayUntil(&LastWakeTime, 10);
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

    TIM5CH1_CAPTURE_STA = 0;
    while (!remoteData.state) {
    }

    // 功能任务
    // xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    // xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 800, NULL, 5, NULL);
    xTaskCreate(Task_Snail, "Task_Snail", 500, NULL, 6, NULL);
    xTaskCreate(Task_Fire, "Task_Fire", 500, NULL, 7, NULL);
    // xTaskCreate(Task_Control, "Task_Control", 500, NULL, 4, NULL);
    // 完成使命
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

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        // Keyboard_Check(&CTRL, &remoteData, KEY_CTRL);

        if (remoteData.switchRight == 1) {
            controlMode = 1; //遥控器模式
        } else if (remoteData.switchRight != 1) {
            controlMode = 2; //键鼠模式
        }
        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}