/**
 * @brief 哨兵
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

int debug1 = 0;
int debug2 = 0;
int debug3 = 0;
int debug4 = 0;
int debug5 = 0;
int debug6 = 0;
int debug7 = 0;
int debug8 = 0;

void Task_Chassis(void *Parameters) {
    TickType_t LastWakeTime   = xTaskGetTickCount(); // 时钟
    float      rpm2rps        = 0.104667f;           // 转子的转速(round/min)换算成角速度(rad/s) = 2 * 3.14 / 60
    float      yawAngleTarget = 0, pitchAngleTarget = 0;
    float      p                      = 0;
    uint32_t   pid                    = 0;
    float      pitchAngleRampProgress = -0.5;

    // 初始化角速度PID
    PID_Init(&PID_Chassis_Left, 1, 0, 0, 4000, 2000);
    PID_Init(&PID_Chassis_Right, 1, 0, 0, 4000, 2000);

    PID_Init(&PID_Stabilizer_Yaw_Angle, 16, 0, 0, 4000, 800); // i 0.06
    PID_Init(&PID_Stabilizer_Yaw_Speed, 14, 0, 0, 800, 2000);

    PID_Init(&PID_Stabilizer_Pitch_Angle, 20, 0.08, 0, 4000, 800);
    PID_Init(&PID_Stabilizer_Pitch_Speed, 8, 0, 0, 3000, 2000);

    while (1) {

        // PS#1
        if (PsData.id != pid) {
            pid = PsData.id;
            yawAngleTarget -= (PsData.result[0] >> 3) - 15;
            pitchAngleTarget -= ((PsData.result[0] & 7) - 3) * CHOOSE(1.5, 2, 2.5);
            MIAO(pitchAngleTarget, -35, 60);
            printf("%d", pitchAngleTarget);
        }

        // PS#2
        // if (PsData.pid != pid) {
        //   pid = PsData.pid;
        //   yawAngleTarget -= PsData.result[0] - 128;
        // }

        // 调试
        // PID_Stabilizer_Pitch_Speed.p = CHOOSE(4, 6, 8);

        // 遥控器
        if (ABS(remoteData.rx) > 10) {
            yawAngleTarget = (float) -1 * remoteData.rx / 16.5;
        }

        if (ABS(remoteData.ry) > 10) {
            pitchAngleTarget = (float) -1 * ABS(remoteData.ry) / 7.33;
        }

        if (pitchAngleRampProgress < 1) {
            pitchAngleRampProgress += 0.01f;
        }

        // 计算输出电流PID:底盘PID
        PID_Calculate(&PID_Chassis_Left, (float) remoteData.lx, Motor_Chassis_Left.speed * rpm2rps);
        PID_Calculate(&PID_Chassis_Right, (float) remoteData.lx, Motor_Chassis_Right.speed * rpm2rps);

        // 计算输出电流PID:Yaw轴PID
        PID_Calculate(&PID_Stabilizer_Yaw_Angle, yawAngleTarget, Motor_Stabilizer_Yaw.angle);
        PID_Calculate(&PID_Stabilizer_Yaw_Speed, PID_Stabilizer_Yaw_Angle.output, Motor_Stabilizer_Yaw.speed * rpm2rps);

        // 计算输出电流PID:Pitch轴PID
        PID_Calculate(&PID_Stabilizer_Pitch_Angle, pitchAngleTarget - RAMP(0.0f, 90.0f, pitchAngleRampProgress), Motor_Stabilizer_Pitch.angle);
        PID_Calculate(&PID_Stabilizer_Pitch_Speed, PID_Stabilizer_Pitch_Angle.output, Motor_Stabilizer_Pitch.speed * rpm2rps);

        // // Yaw轴起转电流
        // if (PID_Stabilizer_Yaw_Angle.error < -0.5 && ABS(Motor_Stabilizer_Yaw.speed) < 5) {
        //     PID_Stabilizer_Yaw_Speed.output = -800;
        // } else if (PID_Stabilizer_Yaw_Angle.error > 0.5 && ABS(Motor_Stabilizer_Yaw.speed) < 5) {
        //     PID_Stabilizer_Yaw_Speed.output = 800;
        // }
        // // Pitch轴起转电流
        // if (PID_Stabilizer_Pitch_Angle.error < -1 && ABS(Motor_Stabilizer_Pitch.speed) < 5) {
        //     PID_Stabilizer_Pitch_Speed.output = -2100;
        // } else if (PID_Stabilizer_Pitch_Angle.error > 1 && ABS(Motor_Stabilizer_Pitch.speed) < 5) {
        //     PID_Stabilizer_Pitch_Speed.output = 0;
        // }

        // 输出电流值到电调
        // Can_Send(CAN1, 0x200, PID_Chassis_Left.output, PID_Chassis_Right.output, PID_Stabilizer_Yaw_Speed.output, PID_Stabilizer_Pitch_Speed.output);
        Can_Send(CAN1, 0x200, 0, 0, PID_Stabilizer_Yaw_Speed.output, PID_Stabilizer_Pitch_Speed.output);

        // debug
        debug1 = yawAngleTarget;
        debug2 = pitchAngleTarget;
        debug3 = RAMP(0.0f, 80.0f, pitchAngleRampProgress);
        debug4 = pitchAngleRampProgress * 1000;

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, 10);
    }

    vTaskDelete(NULL);
}

void Task_Debug_Magic_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        // printf("Yaw: %f \r\n", Gyroscope_EulerData.yaw);
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 500);
    }
    vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

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

    //陀螺仪计数器开启确认

    // 调试任务
#if DEBUG_ENABLED
    Magic_Init_Handle(&magic, 0); // 初始化调试数据的默认值
                                  // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500,
                                  // NULL, 6, NULL); xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send",
                                  // 500, NULL, 6,
                                  //                                                 NULL);
                                  // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6,
                                  // NULL); xTaskCreate(Task_Debug_Gyroscope_Sampling,
                                  // "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);

    // 完成使命
    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}
