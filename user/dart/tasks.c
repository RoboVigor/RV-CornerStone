/**
 * @brief 飞镖
 * @version 1.0.0
 */
#include "main.h"

void Task_Duct(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {

        if (RIGHT_SWITCH_MIDDLE) {
            PWM_Set_Compare(&PWM_Motor_Duct, 7);
        } else {
            PWM_Set_Compare(&PWM_Motor_Duct, 5);
        }

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Servo(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.005;               // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {

        PWM_Set_Compare(&PWM_Servo_Yaw, remoteData.lx / 660.0 * 2.5 + 7.5);   // 5~10
        PWM_Set_Compare(&PWM_Servo_Pitch, remoteData.ry / 660.0 * 2.5 + 7.5); // 5~10

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
        // LED_Run_Horse(); // 跑马灯,建议延时20ms
        vTaskDelayUntil(&LastWakeTime, 200);
#endif
#ifdef STM32F40_41xxx
        LED_Run_Rainbow_Ball(); // 梦幻彩虹灯,建议延时10ms
        vTaskDelayUntil(&LastWakeTime, 10);
#endif
    }
    vTaskDelete(NULL);
}

void Task_Startup_Music(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        if (KTV_Play(Music_XP)) break;
        vTaskDelayUntil(&LastWakeTime, 150);
    }
    vTaskDelete(NULL);
}

void Task_OLED(void *Parameters) {
    uint16_t   JoystickValue = -1;
    TickType_t LastWakeTime  = xTaskGetTickCount();
    oled_init();
    while (1) {
        oled_LOGO();
        vTaskDelayUntil(&LastWakeTime, 125);
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
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_OLED, "Task_OLED", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 运动控制任务
    xTaskCreate(Task_Duct, "Task_Duct", 500, NULL, 5, NULL);
    xTaskCreate(Task_Servo, "Task_Servo", 500, NULL, 5, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
