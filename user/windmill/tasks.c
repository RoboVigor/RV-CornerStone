/**
 * @brief 大风车
 * @version 1.7.0
 */
#include "main.h"

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      interval     = 0.01;            // 任务运行间隔 s
    int        intervalms   = interval * 1000; // 任务运行间隔 ms
    while (1) {
        SafetyMode = (RIGHT_SWITCH_BOTTOM && LEFT_SWITCH_BOTTOM);
        if (RIGHT_SWITCH_TOP) {
            RotateMode = 0;
        } else if (RIGHT_SWITCH_MIDDLE) {
            RotateMode = 1;
        } else if (RIGHT_SWITCH_BOTTOM) {
            RotateMode = 2;
        }
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Windmill(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    int targetSpeed = 0;
    PID_Init(&PID_WindmillSpeed, 100, 0, 0, 1000, 1000);

    while (1) {

        // if (RotateMode == 0) {
        //     targetSpeed = 360;
        // } else if (RotateMode == 1) {
        //     targetSpeed = 0;
        // } else if (RotateMode == 2) {
        //     targetSpeed = -360;
        // }
        targetSpeed = 360;

        // 计算输出电流PID
        PID_Calculate(&PID_WindmillSpeed, targetSpeed, Motor_Windmill.speed * RPM2RPS);

        // 输出电流值到电调(安全起见默认注释此行)
        Motor_Windmill.input = PID_WindmillSpeed.output;
        // DebugData.debug1 = Motor_Windmill.speed;
        // DebugData.debug2 = remoteData.rx;

        // 底盘运动更新频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }

    vTaskDelete(NULL);
}

void Task_Can_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    CAN_TypeDef *Canx[2]          = {CAN1, CAN2};
    Motor_Type **Canx_Device[2]   = {Can1_Device, Can2_Device};
    uint16_t     Can_Send_Id[3]   = {0x200, 0x1ff, 0x2ff};
    uint16_t     Can_ESC_Id[3][4] = {{0x201, 0x202, 0x203, 0x204}, {0x205, 0x206, 0x207, 0x208}, {0x209, 0x020a, 0x20b, 0x20c}};

    int         i, j, k;        // CAN序号 发送ID序号 电调ID序号
    int         isNotEmpty = 0; // 同一发送ID下是否有电机
    Motor_Type *motor;          // 根据i,j,k锁定电机
    int16_t     currents[4];    // CAN发送电流

    while (1) {
        for (i = 0; i < 2; i++) {
            for (j = 0; j < 3; j++) {
                isNotEmpty = 0;
                for (k = 0; k < 4; k++) {
                    motor       = *(Canx_Device[i] + ESC_ID(Can_ESC_Id[j][k]));
                    currents[k] = (motor && motor->inputEnabled) ? motor->input : 0;
                    isNotEmpty  = isNotEmpty || (motor && motor->inputEnabled);
                }
                if (isNotEmpty && !SafetyMode) {
                    Can_Send(Canx[i], Can_Send_Id[j], currents[0], currents[1], currents[2], currents[3]);
                } else if (isNotEmpty && SafetyMode) {
                    Can_Send(Canx[i], Can_Send_Id[j], 0, 0, 0, 0);
                }
            }
        }
        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        LED_Run_Rainbow_Ball();
        vTaskDelayUntil(&LastWakeTime, 10);
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

    //获得 Stone ID
    BSP_Stone_Id_Init(&Board_Id, &Robot_Id);

    // 初始化全局变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 初始化陀螺仪
    // Gyroscope_Init(&Gyroscope_EulerData);

    // 低级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 运动控制任务
    xTaskCreate(Task_Windmill, "Task_Windmill", 400, NULL, 5, NULL);

    // 等待遥控器开启
    // while (!remoteData.state) {
    // }

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
