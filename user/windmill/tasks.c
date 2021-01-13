/**
 * @brief 大风车 C板
 * @version 1.7.0
 */
#include "main.h"

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      interval     = 0.01;            // 任务运行间隔 s
    int        intervalms   = interval * 1000; // 任务运行间隔 ms
    while (1) {
        SafetyMode = (RIGHT_SWITCH_BOTTOM && LEFT_SWITCH_BOTTOM);
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Windmill(void *Parameters) {
    // 任务
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    float targetSpeed         = 0;
    float targetSpeedStart    = 0;
    float targetSpeedStop     = 0;
    float targetSpeedRamp     = 0;
    float targetSpeedProgress = 0;
    float lastTargetSpeed     = 0;

    PID_Init(&PID_WindmillSpeed, 100, 20, 500, 8000, 8000);
    Filter_WindmillSpeed.windowSize = 20;

    while (1) {

        // if (LEFT_SWITCH_TOP) {
        //     targetSpeed = CHOOSER(60, 0, 0);
        // } else if (LEFT_SWITCH_MIDDLE) {
        //     targetSpeed = 0;
        // } else if (LEFT_SWITCH_BOTTOM) {
        //     targetSpeed = -1 * CHOOSER(60, 0, 0);
        // }
        targetSpeed = CHOOSEL(10, 0, 0);
        if (targetSpeed != lastTargetSpeed) {
            PID_WindmillSpeed.output_I = 0;
            targetSpeedStart           = targetSpeedRamp;
            targetSpeedStop            = targetSpeed;
            targetSpeedProgress        = 0;
        }
        lastTargetSpeed     = targetSpeed;
        targetSpeedRamp     = RAMP(targetSpeedStart, targetSpeedStop, targetSpeedProgress);
        targetSpeedProgress = targetSpeedProgress > 1 ? targetSpeedProgress : (targetSpeedProgress + 0.005);
        // PID_WindmillSpeed.i = CHOOSER(5, 20, 40);
        // PID_WindmillSpeed.p = CHOOSER(50, 100, 200);
        // PID_WindmillSpeed.d = CHOOSER(3000, 1000, 200);

        Filter_Update(&Filter_WindmillSpeed, Motor_Windmill.speed / 19.2);
        Filter_Update_Moving_Average(&Filter_WindmillSpeed);

        // 计算输出电流PID
        PID_Calculate(&PID_WindmillSpeed, targetSpeedRamp, Filter_WindmillSpeed.movingAverage);

        // 输出电流值到电调(安全起见默认注释此行)
        Motor_Windmill.input = PID_WindmillSpeed.output;

        DebugData.debug1 = PID_WindmillSpeed.target * 1000;
        DebugData.debug2 = PID_WindmillSpeed.feedback * 1000;
        DebugData.debug3 = PID_WindmillSpeed.output;
        DebugData.debug4 = PID_WindmillSpeed.output_D;

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
#ifdef STM32F427_437xx
        LED_Run_Horse_XP(); // XP开机动画,建议延时200ms
        vTaskDelayUntil(&LastWakeTime, 200);
#endif
#ifdef STM32F40_41xxx
        LED_Run_Rainbow_Ball();
        vTaskDelayUntil(&LastWakeTime, 10);
#endif
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
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 运动控制任务
    xTaskCreate(Task_Windmill, "Task_Windmill", 400, NULL, 5, NULL);

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
