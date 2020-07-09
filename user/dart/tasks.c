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
        LASER_ON;

        if (LEFT_SWITCH_MIDDLE) {
            PWM_Set_Compare(&PWM_Test, 7);
        } else {
            PWM_Set_Compare(&PWM_Test, 5);
        }

        // Can_Send(CAN2, 0x200, 200, 0, 0, 0);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        DebugData.debug1 = remoteData.switchLeft;
    }
    vTaskDelete(NULL);
}

void Task_Board_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    uint16_t id;
    uint16_t dataLength;

    while (1) {

        // 板间通信
        id                                 = 0x501;
        ProtocolData.user.boardAlpha.data1 = 1.11;
        ProtocolData.user.boardAlpha.data2 = 2.22;
        ProtocolData.user.boardAlpha.data3 = 3.33;
        ProtocolData.user.boardAlpha.data4 = 4.44;

        // // USART发送
        // DMA_Disable(USART1_Tx);
        // dataLength = Protocol_Pack(&UserChannel, id);
        // DMA_Enable(USART1_Tx, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);

        // Can发送
        // dataLength = Protocol_Pack(&UserChannel, id);
        // Can_Send_Msg(CAN1, id, UserChannel.sendBuf, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
#ifdef STM32F427_437xx
        // LED_Run_Horse_XP(); // XP开机动画,建议延时200ms
        LED_Run_Horse(); // 跑马灯,建议延时20ms
        vTaskDelayUntil(&LastWakeTime, 20);
#endif
#ifdef STM32F407xx
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

void Task_Sys_Init(void *Parameters) {

    // 初始化全局变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 初始化陀螺仪
    // Gyroscope_Init(&Gyroscope_EulerData);

    // 调试任务
#if DEBUG_ENABLED
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 运动控制任务
    xTaskCreate(Task_Duct, "Task_Duct", 500, NULL, 5, NULL);

    // DMA发送任务
    // xTaskCreate(Task_Board_Communication, "Task_Board_Communication", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
