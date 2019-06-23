/**
 * @brief 甩锅小车
 * @version 0.8.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2 && remoteData.switchLeft == 2) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟

    // 运动模式
    int        mode           = 2;                 // 底盘运动模式,1直线,2转弯
    int        lastMode       = 2;                 // 上一次的运动模式
    float      yawAngleTarget = 0;                 // 目标值
    float      yawAngle, yawSpeed;         // 反馈值

    // PID
    int speed_P           = 30;
    int speed_I           = 0.15;
    int speed_D           = 0;
    int speed_maxOutput   = 10000;
    int speed_maxOutput_I = 4000;

    int yawAngle_P           = 10;
    int yawAngle_maxOutput   = 1000;
    int yawAngle_maxOutput_I = 1000;

    int yawSpeed_P           = 2;
    int yawSpeed_maxOutput   = 4000;
    int yawSpeed_maxOutput_I = 1000;

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, speed_P, speed_I, speed_D, speed_maxOutput, speed_maxOutput_I);
    PID_Init(&PID_LBCM, speed_P, speed_I, speed_D, speed_maxOutput, speed_maxOutput_I);
    PID_Init(&PID_RBCM, speed_P, speed_I, speed_D, speed_maxOutput, speed_maxOutput_I);
    PID_Init(&PID_RFCM, speed_P, speed_I, speed_D, speed_maxOutput, speed_maxOutput_I);

    // 初始化航向角角度PID和角速度PID
    PID_Init(&PID_YawAngle, yawAngle_P, 0, 0, yawAngle_maxOutput, yawAngle_maxOutput_I);
    PID_Init(&PID_YawSpeed, yawSpeed_P, 0, 0, yawSpeed_maxOutput, yawSpeed_maxOutput_I);

    while (1) {

        // 更新运动模式
        mode = ABS(remoteData.rx) < 5 ? 1 : 2;

        // 设置反馈值
        yawAngle = Gyroscope_EulerData.yaw; // 航向角角度反馈
        yawSpeed = ImuData.gz / GYROSCOPE_LSB;       // 航向角角速度反馈

        // 切换运动模式
        if (mode != lastMode) {
            PID_YawAngle.output_I = 0;            // 清空角度PID积分
            PID_YawSpeed.output_I = 0;            // 清空角速度PID积分
            yawAngleTarget        = yawAngle; // 更新角度PID目标值
            lastMode              = mode;         // 更新lastMode
        }

        // 根据运动模式计算PID
        if (mode == 1) {
            PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngle);      // 计算航向角角度PID
            PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeed); // 计算航向角角速度PID
        } else {
            PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeed); // 计算航向角角速度PID
        }

        // 设置底盘总体移动速度
        Chassis_Update(&ChassisData, (float) -remoteData.lx / 330.0f, (float) remoteData.ly / 330.0f, (float) PID_YawSpeed.output / 660.0f);

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Limit_Rotor_Speed(&ChassisData, 300);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, ChassisData.rotorSpeed[0], Motor_LF.speed * RPM2RPS);
        PID_Calculate(&PID_LBCM, ChassisData.rotorSpeed[1], Motor_LB.speed * RPM2RPS);
        PID_Calculate(&PID_RBCM, ChassisData.rotorSpeed[2], Motor_RB.speed * RPM2RPS);
        PID_Calculate(&PID_RFCM, ChassisData.rotorSpeed[3], Motor_RF.speed * RPM2RPS);

        // 输出电流值到电调(安全起见默认注释此行)
        // Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);


        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Take(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {

        if (remoteData.switchLeft == 1) {

            vTaskDelay(800);
            takeMode = 0;        // 传动伸出
            ROTATE_ON;
            vTaskDelay(500);
            TAKE_ON;
            vTaskDelay(800);
            vTaskDelay(1500);
            ROTATE_OFF;
            vTaskDelay(500);
            ROTATE_ON;
            vTaskDelay(500);
            TAKE_OFF;
            ROTATE_OFF;
            vTaskDelay(500);
            takeMode = 1;    // 传动返回
            vTaskDelay(4000);
            
        }

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 传动装置
 * @keep
 *  + Motor_Tansmission 2006 CAN1 ID:7
 *  + 电流方向：正 往回
 */

void Task_Take_Vertical(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int   targetAngle = -1200;
    int   lastSpeed   = 0;
    int   lastAngle   = 0;

    PID_Init(&PID_TV_Angle, 6, 0, 0, 4000, 2000);
    PID_Init(&PID_TV_Speed, 2, 0, 0, 4000, 2000);
    
    while (1) {
        lastSpeed         = Motor_TV.speed;
        lastAngle         = Motor_TV.angle;

        // if (takeMode == 0) {
        //     targetAngle = 300;
        // } else if (takeMode == 1) {
        //     targetAngle = 0;
        // } 

        PID_Calculate(&PID_TV_Angle, targetAngle, lastAngle);
        PID_Calculate(&PID_TV_Speed, PID_TV_Angle.output, lastSpeed * RPM2RPS);

        Can_Send(CAN1, 0x1FF, 0, PID_TV_Speed.output, 0, 0);
        // Can_Send(CAN1, 0x1FF, 0, -1000, 0, 0);

        DebugA=PID_TV_Speed.output;
        DebugB= targetAngle;
        DebugC=Motor_TV.angle;


        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Landing(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    uint8_t    bangMode     = 0; // 上岛传感器切换， 0 前端， 1 后端
    uint8_t    powerMode    = 0; // 电源

    // PID 初始化
    PID_Init(&PID_LGW, 5, 0, 0, 4000, 2000);
    PID_Init(&PID_RGW, 5, 0, 0, 4000, 2000);

    while (1) {
    
        // if (remoteData.switchLeft == 3) {
        //     LANDING_SWITCH_FRONT;
        //     LANDING_SWITCH_FRONT2;
        // } else if (remoteData.switchLeft == 1) {
        //     LANDING_SWITCH_BEHIND;
        //     LANDING_SWITCH_BEHIND2;
        // }

        // // Power Control
        // if (remoteData.switchRight == 1) {
        //     LANDING_POWER_ON;
        // } else {
        //     LANDING_POWER_OFF;
        // }

        // PID 计算
        PID_Increment_Calculate(&PID_LGW, remoteData.ry, Motor_LGW.speed * RPM2RPS);
        PID_Increment_Calculate(&PID_RGW, -remoteData.ry, Motor_RGW.speed * RPM2RPS);

        // 发送数据至电调
        Can_Send(CAN2, 0x200, PID_LGW.output, PID_RGW.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Supply(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        // Controller
        if (remoteData.switchLeft == 1) {
            PWM_Set_Compare(&PWM_Supply1, 15);
            PWM_Set_Compare(&PWM_Supply2, 15);
        } else {
            PWM_Set_Compare(&PWM_Supply1, 5);
            PWM_Set_Compare(&PWM_Supply2, 25);
        }

        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

void Task_Rescue(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    uint8_t rescueMode = 0; // 0 救援爪抬起 1 救援爪放下

    while (1) {

        if (remoteData.switchRight = 1) {
            RESCUE_HOOK_UP;
        } else {
            RESCUE_HOOK_DOWN;
        }

        vTaskDelayUntil(&LastWakeTime, 50);
    }

    vTaskDelete(NULL);
}

void Task_Distance_Sensor(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    // 通过PWM波读取距离信息
    uint16_t   temp1 = 0;
    uint16_t   temp2 = 0;
    uint16_t   last_distance = 0;
    int        i = 0;
    Distance1       = 0;
    Distance2       = 0;

    while (1) {
            temp1 = TIM2CH1_CAPTURE_VAL; //得到总的高电平时间

            if (Distance1 == 0) {     // 第一次
                Distance1 = temp1 / 100; // cm us
            }

            if (ABS((temp1/100)-Distance1) <= 20) {
                Distance1 = temp1 / 100; // cm us
            }


            temp2 = TIM5CH1_CAPTURE_VAL; //得到总的高电平时间

            if (Distance2 == 0) {     // 第一次
                Distance2 = temp2 / 100; // cm us
            }

            if (ABS((temp2/100)-Distance2) <= 20) {
                Distance2 = temp2 / 100; // cm us
            }

        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Upthrow(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    float upthrowAngleTarget = 0;
    float upthrowProgress = 0;
    float upthrowStart = 0;

    PID_Init(&PID_Upthrow1_Speed, 15, 0, 0, 4000, 2000);  // 30
    PID_Init(&PID_Upthrow1_Angle, 1.0, 0, 0, 4000, 2000); // 1.4

    while (1) {

        upthrowAngleTarget = RAMP(Motor_Upthrow1.angle, 1000, upthrowProgress);
        if (upthrowProgress < 1) {
            upthrowProgress += 0.005f;
        }

        PID_Calculate(&PID_Upthrow1_Angle, upthrowAngleTarget, Motor_Upthrow1.angle);
        PID_Calculate(&PID_Upthrow1_Speed, PID_Upthrow1_Angle.output, Motor_Upthrow1.speed * RPM2RPS);

        Can_Send(CAN1, 0x1FF, PID_Upthrow1_Speed.output, -PID_Upthrow1_Speed.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 2);
    }
    vTaskDelete(NULL);
}

void Task_Optoelectronic_Input_Take(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int last_take_state1 = 0;
    int last_take_state2 = 0;
    int last_take_state3 = 0;
    int last_take_state4 = 0;
    int take_state1 = 0;
    int take_state2 = 0;
    int take_state3 = 0;
    int take_state4 = 0;


    while(1) {
        // 取弹
        take_state1 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_7);
        if (last_take_state1 != take_state1) {
            vTaskDelay(10);
            if (take_state1 == GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_7)) {
                T_State1 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_7);
                last_take_state1 = take_state1; 
            }
        }
        take_state2 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_6);
        if (last_take_state2 != take_state2) {
            vTaskDelay(10);
            if (take_state2 == GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_6)) {
                T_State2 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_6);
                last_take_state2 = take_state2; 
            }
        }
        take_state3 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_5);
        if (last_take_state3 != take_state3) {
            vTaskDelay(10);
            if (take_state3 == GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_5)) {
                T_State3 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_5);
                last_take_state3 = take_state3; 
            }
        }
        take_state4 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_2);
        if (last_take_state4 != take_state4) {
            vTaskDelay(10);
            if (take_state4 == GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_2)) {
                T_State4 = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_2);
                last_take_state4 = take_state4; 
            }
        }
        
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Optoelectronic_Input_Landing(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int last_landing_front_state1 = 0;
    int last_landing_front_state2 = 0;
    int last_landing_behind_state1 = 0;
    int last_landing_behind_state2 = 0;
    int landing_front_state1 = 0;
    int landing_front_state2 = 0;
    int landing_behind_state1 = 0;
    int landing_behind_state2 = 0;

    while(1) {
        // 登岛
        landing_front_state1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5);
        if (last_landing_front_state1 != landing_front_state1) {
            vTaskDelay(10);
            if (landing_front_state1 == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) {
                LF_State1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5);
                last_landing_front_state1 = landing_front_state1; 
            }
        }
        landing_front_state2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6);
        if (last_landing_front_state2 != landing_front_state2) {
            vTaskDelay(10);
            if (landing_front_state2 == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) {
                LF_State2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6);
                last_landing_front_state2 = landing_front_state2; 
            }
        }
        landing_behind_state1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
        if (last_landing_behind_state1 != landing_behind_state1) {
            vTaskDelay(10);
            if (landing_behind_state1 == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)) {
                LB_State1 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4);
                last_landing_behind_state1 = landing_behind_state1; 
            }
        }
        landing_behind_state2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12);
        if (last_landing_behind_state2 != landing_behind_state2) {
            vTaskDelay(10);
            if (landing_behind_state2 == GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12)) {
                LB_State2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12);
                last_landing_behind_state2 = landing_behind_state2; 
            }
        }
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Take_Horizontal(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    float TargetSpeed = 0;

    PID_Init(&PID_TH_Speed, 25, 0.1, 0, 2000, 2000);

    while (1) {

        if(T_State1 == 1 && T_State2 == 0 && T_State3 == 0 && T_State4 == 1) {
            TargetSpeed = 0;
        } else {
            TargetSpeed = -350;
        }

        PID_Calculate(&PID_TH_Speed, TargetSpeed, Motor_TH.speed * RPM2RPS);

        Can_Send(CAN2, 0x200, 0, PID_TH_Speed.output, 0, 0);

        DebugA=TargetAngle;

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Limit_Switch(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    int last_limit_switch_left = 0;
    int limit_switch_left = 0;
    int last_limit_switch_right = 0;
    int limit_switch_right = 0;

    while(1) {
        limit_switch_left = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
        if (last_limit_switch_left != limit_switch_left) {
            vTaskDelay(10);
            if (limit_switch_left == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2)) {
                LSL_State = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
                last_limit_switch_left = limit_switch_left; 
            }
        }
        limit_switch_right = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
        if (last_limit_switch_right != limit_switch_right) {
            vTaskDelay(10);
            if (limit_switch_right == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3)) {
                LSR_State = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
                last_limit_switch_right = limit_switch_right; 
            }
        }

        vTaskDelayUntil(&LastWakeTime, 10);
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
        if (KTV_Play(Music_Soul)) break;
        vTaskDelayUntil(&LastWakeTime, 60);
    }

    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

    // 初始化局部变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 初始化陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData);

    // 调试任务
#if DEBUG_ENABLED
    xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // Structure
    // xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Distance_Sensor, "Task_Distance_Sensor", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Take, "Task_Take", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Landing, "Task_Landing", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Supply, "Task_Supply", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Rescue, "Task_Rescue", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Take_Horizontal, "Task_Take_Horizontal", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Take_Vertical, "Task_Take_Vertical", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Optoelectronic_Input_Take, "Task_Optoelectronic_Input_Take", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Optoelectronic_Input_Landing, "Task_Optoelectronic_Input_Landing", 400, NULL, 3, NULL);
    xTaskCreate(Task_Upthrow,  "Task_Upthrow", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Limit_Switch,  "Task_Limit_Switch", 400, NULL, 3, NULL);
    /* End */

    // 完成使命
    vTaskDelete(NULL);
    vTaskDelay(10);
}