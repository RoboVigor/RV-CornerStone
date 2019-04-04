/**
 * @brief 甩锅小车
 * @version 0.8.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2) {
            Can_Send(CAN1, 0x200, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}

void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    int        rotorSpeed[4];                      // 轮子转速
    float      rpm2rps        = 0.104667f;         // 转子的转速(round/min)换算成角速度(rad/s) = 2 * 3.14 / 60
    int        mode           = 2;                 // 底盘运动模式,1直线,2转弯
    int        lastMode       = 2;                 // 上一次的运动模式
    float      yawAngleTarget = 0;                 // 目标值
    float      yawAngleFeed, yawSpeedFeed;         // 反馈值

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, 19, 0.15, 0, 4000, 2000);
    PID_Init(&PID_LBCM, 19, 0.15, 0, 4000, 2000);
    PID_Init(&PID_RBCM, 19, 0.15, 0, 4000, 2000);
    PID_Init(&PID_RFCM, 19, 0.15, 0, 4000, 2000);

    // 初始化航向角角度PID和角速度PID
    PID_Init(&PID_YawAngle, 10, 0, 0, 1000, 1000);
    PID_Init(&PID_YawSpeed, 2, 0, 0, 4000, 1000);

    while (1) {

        // 更新运动模式
        mode = ABS(remoteData.rx) < 5 ? 1 : 2;

        // 设置反馈值
        yawAngleFeed = Gyroscope_EulerData.yaw; // 航向角角度反馈
        yawSpeedFeed = mpu6500_data.gz / 16.4;  // 航向角角速度反馈

        // 切换运动模式
        if (mode != lastMode) {
            PID_YawAngle.output_I = 0;            // 清空角度PID积分
            PID_YawSpeed.output_I = 0;            // 清空角速度PID积分
            yawAngleTarget        = yawAngleFeed; // 更新角度PID目标值
            lastMode              = mode;         // 更新lastMode
        }

        // 根据运动模式计算PID
        if (mode == 1) {
            PID_Calculate(&PID_YawAngle, yawAngleTarget, yawAngleFeed);      // 计算航向角角度PID
            PID_Calculate(&PID_YawSpeed, PID_YawAngle.output, yawSpeedFeed); // 计算航向角角速度PID
        } else {
            PID_Calculate(&PID_YawSpeed, -remoteData.rx, yawSpeedFeed); // 计算航向角角速度PID
        }

        // 设置底盘总体移动速度
        Chassis_Update(&ChassisData, (float) -remoteData.lx / 660.0f, (float) remoteData.ly / 660.0f, (float) PID_YawSpeed.output / 1320.0f);

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Get_Rotor_Speed(&ChassisData, rotorSpeed);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
        PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
        PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
        PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

        // 输出电流值到电调(安全起见默认注释此行)
        Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

        DebugA = Motor_LF.speed;

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Take(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      rpm2rps      = 3.14 / 60; // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    float      target       = -180;
    float      targetAngle  = target;
    float      lastAngle    = 0;
    int        cnt          = 0;
    int        waiting      = 0;

    PID_Init(&PID_TakeLeft_Speed, 6, 0, 0, 4000, 2000);
    PID_Init(&PID_TakeRight_Speed, 6, 0, 0, 4000, 2000);
    PID_Init(&PID_TakeLeft_Angle, 14, 0, 0, 4000, 2000);
    PID_Init(&PID_TakeRight_Angle, 14, 0, 0, 4000, 2000);

    while (1) {

        if (remoteData.switchLeft == 1) {
            // TAKE_ON;
        } else if (remoteData.switchLeft == 2) {
            // TAKE_OFF;
        }

        if (remoteData.switchLeft == 3) {
            // BANG_ON;
        } else {
            // BANG_OFF;
        }

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Transmission(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      rpm2rps      = 3.14 / 60; // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    float      target       = -30000;
    float      targetAngle  = target;
    float      lastAngle    = 0;
    int        rotateMode   = 1; // 0: ，1:

    PID_Init(&PID_Transmission_Speed, 10, 0, 0, 4000, 2000);
    PID_Init(&PID_Transmission_Angle, 1, 0, 0, 4000, 2000);

    while (1) {

        // 读取电机角度值
        lastAngle = Motor_Transmission.angle;

        // 控制代码
        // 置位 3 - 向内传动
        // 置位 1 - 向外传动
        if (remoteData.switchRight == 3) {
            targetAngle = 0;
            if (lastAngle > -20000 && Motor_Transmission.speed == 0) {
                Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);
            } else {
                Can_Send(CAN1, 0x1FF, 0, 0, 2000, 0);
            }
        } else if (remoteData.switchRight == 1) {
            targetAngle = target;
            if (lastAngle < -20000 && Motor_Transmission.speed == 0) {
                Can_Send(CAN1, 0x1FF, 0, 0, 0, 0);
            } else {
                Can_Send(CAN1, 0x1FF, 0, 0, -2000, 0);
            }
        }

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_GuideWheel(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      rpm2rps      = 3.14 / 60;           // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)

    // PID 初始化
    PID_Init(&PID_LGW, 5, 0, 0, 4000, 2000);
    PID_Init(&PID_RGW, 5, 0, 0, 4000, 2000);

    while (1) {

        // PID 计算
        PID_Increment_Calculate(&PID_LGW, remoteData.ry, Motor_LGW.speed * rpm2rps);
        PID_Increment_Calculate(&PID_RGW, -remoteData.ry, Motor_RGW.speed * rpm2rps);

        // 发送数据至电调
        // Can_Send(CAN2, 0x200, PID_LGW.output, PID_RGW.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_BANG(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    uint8_t    bangMode     = 0;
    uint8_t    powerMode    = 0;

    while (1) {

        // Monitor
        if (remoteData.switchLeft == 3) {
            bangMode = 0;
        } else if (remoteData.switchLeft == 1) {
            bangMode = 1;
        }
        if (remoteData.switchRight == 3) {
            powerMode = 0;
        } else if (remoteData.switchRight == 1) {
            powerMode = 1;
        }

        // Switch Control
        if (bangMode == 0) {
            LANDING_SWITCH_FRONT;
        } else if (bangMode == 1) {
            LANDING_SWITCH_BEHIND;
        }

        // Power Control
        if (powerMode == 0) {
            LANDING_POWER_OFF;
        } else if (powerMode == 1) {
            LANDING_POWER_ON;
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

void Task_Distance_Sensor(void *Parameter) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    // 通过PWM波读取距离信息
    extern u8  TIM2CH_CAPTURE_STA[2]; //输入捕获状态 
    extern u32 TIM2CH_CAPTURE_VAL[2]; //输入捕获值 
    uint16_t distance1 = 0;
    uint16_t distance2 = 0;
    uint16_t temp = 0; 

    while (1) {
        //CH1成功捕获到了一次高电平 
        if(TIM2CH_CAPTURE_STA[0]&0X80) {
            // temp=TIM2CH_CAPTURE_STA[0]&0X3F; 
            // temp*=0XFFFFFFFF; //溢出时间总和
            temp=TIM2CH_CAPTURE_VAL[0]; //得到总的高电平时间 

            if (temp < 40000 && temp > 500) {
                distance1 = temp / 100; // cm us
            }

            DebugZ = distance1;
					
            TIM2CH_CAPTURE_STA[0]=0; // 开启下一次捕获
        }

        //CH2成功捕获到了一次高电平 
        if(TIM2CH_CAPTURE_STA[1]&0X80) {
            // temp=TIM2CH_CAPTURE_STA[0]&0X3F; 
            // temp*=0XFFFFFFFF; //溢出时间总和
            temp=TIM2CH_CAPTURE_VAL[1]; //得到总的高电平时间 

            if (temp < 40000 && temp > 500) {
                distance2 = temp / 100; // cm us
            }

            DebugY = distance2;
					
            TIM2CH_CAPTURE_STA[1]=0; // 开启下一次捕获
        }
        
        vTaskDelayUntil(&LastWakeTime, 10);
        
    }

    // 串口2读取传感器MCU信息
    // u8        sum = 0, i = 0;
    // u8        RangeStatus = 0, Time = 0, Mode = 0;
    // int16_t   data     = 0;
    // uint16_t  distance = 0;
    // extern u8 re_buf_Data[8], receive_ok;

    // while (1) {

    //     if (receive_ok) //串口接收完毕
    //     {
    //         for (sum = 0, i = 0; i < (re_buf_Data[3] + 4); i++) // rgb_data[3]=3
    //             sum += re_buf_Data[i];
    //         if (sum == re_buf_Data[i]) //校验和判断
    //         {
    //             distance    = re_buf_Data[4] << 8 | re_buf_Data[5];
    //             RangeStatus = (re_buf_Data[6] >> 4) & 0x0f;
    //             Time        = (re_buf_Data[6] >> 2) & 0x03;
    //             Mode        = re_buf_Data[6] & 0x03;
    //         }
    //         receive_ok = 0; //处理数据完毕标志
    //     }

    //     vTaskDelayUntil(&LastWakeTime, 10);
    // }

    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {
    // 进入临界区
    taskENTER_CRITICAL();

    // 初始化全局变量
    Handle_Init();

    // BSP们
    BSP_GPIO_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_DMA_Init();
    BSP_TIM_Init();
    BSP_NVIC_Init();
    BSP_USER_Init();

    // 初始化陀螺仪
    MPU6500_Initialize();
    MPU6500_EnableInt();

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // 调试任务
#if DEBUG_ENABLED
    Magic_Init_Handle(&magic, 0); // 初始化调试数据的默认值
    xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // GREEN_LIGHT_ON;

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    xTaskCreate(Task_Take, "Task_Take", 400, NULL, 3, NULL);
    xTaskCreate(Task_GuideWheel, "Task_GuideWheel", 400, NULL, 3, NULL);
    xTaskCreate(Task_BANG, "Task_BANG", 400, NULL, 3, NULL);
    xTaskCreate(Task_Transmission, "Task_Transmission", 400, NULL, 3, NULL);
    xTaskCreate(Task_Distance_Sensor, "Task_Distance_Sensor", 400, NULL, 3, NULL);

    // 完成使命
    vTaskDelete(NULL);

    // 退出临界区
    taskEXIT_CRITICAL();
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}
