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
    int        rotorSpeed[4];                      // 轮子转速
    float      rpm2rps        = 0.104667f;         // 转子的转速(round/min)换算成角速度(rad/s) = 2 * 3.14 / 60
    int        mode           = 2;                 // 底盘运动模式,1直线,2转弯
    int        lastMode       = 2;                 // 上一次的运动模式
    float      yawAngleTarget = 0;                 // 目标值
    float      yawAngleFeed, yawSpeedFeed;         // 反馈值

    float debug_p = 30;
    float debug_i = 0.15;

    // 初始化麦轮角速度PID
    PID_Init(&PID_LFCM, debug_p, debug_i, 0, 10000, 4000); // 19 0.15
    PID_Init(&PID_LBCM, debug_p, debug_i, 0, 10000, 4000);
    PID_Init(&PID_RBCM, debug_p, debug_i, 0, 10000, 4000);
    PID_Init(&PID_RFCM, debug_p, debug_i, 0, 10000, 4000);

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
            PID_Calculate(&PID_YawAngle, yawAngleTarget,
                          yawAngleFeed); // 计算航向角角度PID
            PID_Calculate(&PID_YawSpeed, PID_YawAngle.output,
                          yawSpeedFeed); // 计算航向角角速度PID
        } else {
            PID_Calculate(&PID_YawSpeed, -remoteData.rx,
                          yawSpeedFeed); // 计算航向角角速度PID
        }

        // 设置底盘总体移动速度
        // Chassis_Update(&ChassisData, -(float) remoteData.lx / 660.0f, (float) remoteData.ly / 660.0f, (float) PID_YawSpeed.output / 1320.0f);
        Chassis_Update(&ChassisData, -(float) remoteData.lx / 330.0f, (float) remoteData.ly / 330.0f, (float) PID_YawSpeed.output / 660.0f);

        // 麦轮解算&限幅,获得轮子转速
        Chassis_Get_Rotor_Speed(&ChassisData, rotorSpeed);

        // 计算输出电流PID
        PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
        PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
        PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
        PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

        // 输出电流值到电调(安全起见默认注释此行)
        Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

// void Task_Chassis(void *Parameters) {
//     TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
//     int        rotorSpeed[4];                      // 轮子转速
//     float      rpm2rps        = 0.104667f;         // 转子的转速(round/min)换算成角速度(rad/s) = 2 * 3.14 / 60
//     int        mode           = 2;                 // 底盘运动模式,1直线,2转弯
//     int        lastMode       = 2;                 // 上一次的运动模式
//     float      yawAngleTarget = 0;                 // 目标值
//     float      yawAngleFeed, yawSpeedFeed;         // 反馈值
//     float      x_moving = 0;                       // 键盘x轴方向移动
//     float      y_moving = 0;                       // 键盘y轴方向移动
//     // float      mouse_rotating = 0;                 // 鼠标x轴控制旋转

//     float debug_p = 30;
//     float debug_i = 0;

//     // 初始化麦轮角速度PID
//     PID_Init(&PID_LFCM, debug_p, debug_i, 0, 4000, 2000); // 19 0.15
//     PID_Init(&PID_LBCM, debug_p, debug_i, 0, 4000, 2000);
//     PID_Init(&PID_RBCM, debug_p, debug_i, 0, 4000, 2000);
//     PID_Init(&PID_RFCM, debug_p, debug_i, 0, 4000, 2000);

//     // 初始化航向角角度PID和角速度PID
//     PID_Init(&PID_YawAngle, 10, 0, 0, 1000, 1000);
//     PID_Init(&PID_YawSpeed, 2, 0, 0, 4000, 1000);

//     while (1) {

//         // 更新运动模式
//         mode = remoteData.mouse.x < 5 ? 1 : 2;

//         // 设置反馈值
//         yawAngleFeed = Gyroscope_EulerData.yaw; // 航向角角度反馈
//         yawSpeedFeed = mpu6500_data.gz / 16.4;  // 航向角角速度反馈

//         // 切换运动模式
//         if (mode != lastMode) {
//             PID_YawAngle.output_I = 0;            // 清空角度PID积分
//             PID_YawSpeed.output_I = 0;            // 清空角速度PID积分
//             yawAngleTarget        = yawAngleFeed; // 更新角度PID目标值
//             lastMode              = mode;         // 更新lastMode
//         }

//         // 根据运动模式计算PID
//         if (mode == 1) {
//             PID_Calculate(&PID_YawAngle, yawAngleTarget,
//                           yawAngleFeed); // 计算航向角角度PID
//             PID_Calculate(&PID_YawSpeed, PID_YawAngle.output,
//                           yawSpeedFeed); // 计算航向角角速度PID
//         } else {
//             PID_Calculate(&PID_YawSpeed, remoteData.mouse.x,
//                           yawSpeedFeed); // 计算航向角角速度PID
//         }

//         if (remoteData.keyBoard.keyCode == KEY_A) {
//             x_moving = 1;
//         } else if (remoteData.keyBoard.keyCode == KEY_D) {
//             x_moving = -1;
//         } else {
//             x_moving = 0;
//         }

//         if (remoteData.keyBoard.keyCode == KEY_W) {
//             y_moving = 1;
//         } else if (remoteData.keyBoard.keyCode == KEY_S) {
//             y_moving = -1;
//         } else {
//             y_moving = 0;
//         }

//         // 设置底盘总体移动速度
//         // Chassis_Update(&ChassisData, -(float) remoteData.lx / 660.0f, (float) remoteData.ly / 660.0f, (float) PID_YawSpeed.output / 1320.0f);
//         Chassis_Update(&ChassisData, -(float) x_moving * 2, (float) y_moving * 2, (float) PID_YawSpeed.output / 220.0f);
//         // Chassis_Update(&ChassisData, -(float) remoteData.lx / 660.0f, (float) remoteData.ly / 660.0f, 0);

//         // 麦轮解算&限幅,获得轮子转速
//         Chassis_Get_Rotor_Speed(&ChassisData, rotorSpeed);

//         // 计算输出电流PID
//         PID_Calculate(&PID_LFCM, rotorSpeed[0], Motor_LF.speed * rpm2rps);
//         PID_Calculate(&PID_LBCM, rotorSpeed[1], Motor_LB.speed * rpm2rps);
//         PID_Calculate(&PID_RBCM, rotorSpeed[2], Motor_RB.speed * rpm2rps);
//         PID_Calculate(&PID_RFCM, rotorSpeed[3], Motor_RF.speed * rpm2rps);

//         // 输出电流值到电调(安全起见默认注释此行)
//         Can_Send(CAN1, 0x200, PID_LFCM.output, PID_LBCM.output, PID_RBCM.output, PID_RFCM.output);
//         // Can_Send(CAN1, 0x200, 2000, 2000, 2000, 2000);

//         DebugA = Motor_LF.speed;
//         DebugB = Motor_LB.speed;
//         DebugC = Motor_RB.speed;
//         DebugD = Motor_RF.speed;

//         // 底盘运动更新频率
//         vTaskDelayUntil(&LastWakeTime, 10);
//     }

//     vTaskDelete(NULL);
// }

void Task_Take(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      rpm2rps      = 3.14 / 60; // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)

    while (1) {

        if (remoteData.switchLeft == 1) {

            TAKING_ROD_POWER_ON;
            TAKING_ROD_PUSH;
            vTaskDelay(800);
            TAKING_ROD_POWER_OFF;
            takeMode = 0;        // 传动伸出
            ROTATE_ON;
            vTaskDelay(500);
            TAKE_ON;
            vTaskDelay(800);
            TAKING_ROD_POWER_ON;
            vTaskDelay(1500);
            TAKING_ROD_POWER_OFF;
            ROTATE_OFF;
            vTaskDelay(500);
            ROTATE_ON;
            vTaskDelay(500);
            TAKE_OFF;
            ROTATE_OFF;
            vTaskDelay(500);
            takeMode = 1;    // 传动返回
            TAKING_ROD_POWER_ON;
            TAKING_ROD_PULL;
            vTaskDelay(4000);
            
        }

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Taking_Transmission(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      rpm2rps      = 3.14 / 60; // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)
    int   targetAngle = 0;
    int   lastSpeed   = 0;
    int   lastAngle   = 0;

    PID_Init(&PID_Transmission_Speed, 85, 0, 0, 4000, 2000);
    PID_Init(&PID_Transmission_Angle, 5, 0, 0, 4000, 2000);

    while (1) {

        if (takeMode == 0) {
            targetAngle = 300;
        } else if (takeMode == 1) {
            targetAngle = 0;
        }

        lastSpeed         = Motor_Transmission.speed;
        lastAngle         = Motor_Transmission.angle;
        PID_Calculate(&PID_Transmission_Angle, targetAngle, lastAngle);
        PID_Calculate(&PID_Transmission_Speed, PID_Transmission_Angle.output, lastSpeed * rpm2rps);
        Can_Send(CAN1, 0x1FF, 0, 0, PID_Transmission_Speed.output, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Landing(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      rpm2rps      = 3.14 / 60;           // 转子的转速(RPM,RoundPerMinute)换算成角速度(RadPerSecond)

    // PID 初始化
    PID_Init(&PID_LGW, 5, 0, 0, 4000, 2000);
    PID_Init(&PID_RGW, 5, 0, 0, 4000, 2000);

    while (1) {

        // Switch Control
        if (remoteData.switchLeft == 3) {
            LANDING_SWITCH_FRONT;
            LANDING_SWITCH_FRONT2;
        } else if (remoteData.switchLeft == 1) {
            LANDING_SWITCH_BEHIND;
            LANDING_SWITCH_BEHIND2;
        }

        // Power Control
        if (remoteData.switchRight == 1) {
            LANDING_POWER_ON;
        } else {
            LANDING_POWER_OFF;
        }

        // PID 计算
        PID_Increment_Calculate(&PID_LGW, remoteData.ry, Motor_LGW.speed * rpm2rps);
        PID_Increment_Calculate(&PID_RGW, -remoteData.ry, Motor_RGW.speed * rpm2rps);

        // 发送数据至电调
        Can_Send(CAN2, 0x200, PID_LGW.output, PID_RGW.output, 0, 0);

        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}

void Task_Supply(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {

        if (remoteData.switchLeft == 1) {
            Turn_Supply(1);
        } else {
            Turn_Supply(0);
        }

        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

void Task_Rescue(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

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

// void Task_Debug_Magic_Send(void *Parameters) {
//     TickType_t LastWakeTime = xTaskGetTickCount();

//     while (1) {
//         taskENTER_CRITICAL(); // 进入临界段
//         printf("Yaw: %f \r\n", Gyroscope_EulerData.yaw);
//         taskEXIT_CRITICAL(); // 退出临界段
//         vTaskDelayUntil(&LastWakeTime, 500);
//     }
//     vTaskDelete(NULL);
// }

// void Task_Distance_Sensor(void *Parameter) {
//     TickType_t LastWakeTime = xTaskGetTickCount();
//     // 串口2读取传感器MCU信息
//     u8        sum = 0, i = 0;
//     u8        RangeStatus = 0, Time = 0, Mode = 0;
//     int16_t   data     = 0;
//     uint16_t  distance = 0;
//     extern u8 re_buf_Data[8], receive_ok;

//     while (1) {

//         if (receive_ok) //串口接收完毕
//         {
//             for (sum = 0, i = 0; i < (re_buf_Data[3] + 4); i++) {
//                 sum += re_buf_Data[i];
//             }

//             if (sum == re_buf_Data[i]) //校验和判断
//             {
//                 distance    = re_buf_Data[4] << 8 | re_buf_Data[5];
//                 RangeStatus = (re_buf_Data[6] >> 4) & 0x0f;
//                 Time        = (re_buf_Data[6] >> 2) & 0x03;
//                 Mode        = re_buf_Data[6] & 0x03;
//             }
//             receive_ok = 0; //处理数据完毕标志
//         }

//         vTaskDelayUntil(&LastWakeTime, 10);
//     }

//     vTaskDelete(NULL);
// }

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
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6,
    // NULL); xTaskCreate(Task_Debug_Gyroscope_Sampling,
    // "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // GREEN_LIGHT_ON;

    // 功能任务
    // Base
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // Chassis
    // xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    // Landing
    xTaskCreate(Task_Landing, "Task_Landing", 400, NULL, 3, NULL);
    // Take
    // xTaskCreate(Task_Take, "Task_Take", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Taking_Transmission, "Task_Taking_Transmission", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Distance_Sensor, "Task_Distance_Sensor", 400, NULL, 3, NULL);
    // Supply
    // xTaskCreate(Task_Supply, "Task_Supply", 400, NULL, 3, NULL);
    // Rescue
    // xTaskCreate(Task_Rescue, "Task_Rescue", 400, NULL, 3, NULL);

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