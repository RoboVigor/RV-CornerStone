/**
 * @brief 甩锅小车
 * @version 1.2.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
    while (1) {
        if (remoteData.switchRight == 2) {
            vTaskSuspendAll();
            while (1) {
                Can_Send(CAN1, 0x200, 0, 0, 0, 0);
                vTaskDelay(2);
            }
        }
        vTaskDelay(2);
    }
    vTaskDelete(NULL);
}

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        ChargeEnabled  = RIGHT_SWITCH_BOTTOM;
        ChargeDisabled = RIGHT_SWITCH_MIDDLE;
        HookClose      = LEFT_SWITCH_MIDDLE;
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

void Task_Board_Communication(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.1;                 // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        uint16_t id;
        uint16_t dataLength;
        uint16_t offset = 0;

        // 板间通信
#ifdef BOARD_ALPHA
        id                                 = 0x501;
        ProtocolData.user.boardAlpha.data1 = 1.11;
        ProtocolData.user.boardAlpha.data2 = 2.22;
        ProtocolData.user.boardAlpha.data3 = 3.33;
        ProtocolData.user.boardAlpha.data4 = 4.44;
#endif

#ifdef BOARD_BETA
        id                                = 0x502;
        ProtocolData.user.boardBeta.data1 = 0;
        ProtocolData.user.boardBeta.data2 = 0;
        ProtocolData.user.boardBeta.data3 = 0;
        ProtocolData.user.boardBeta.data4 = 1.11;
#endif

        // USART发送
        DMA_Disable(UART7_Tx);
        Protocol_Get_Packet_Info(id, &offset, &dataLength);
        dataLength = Protocol_Pack(&UserChannel, id);
        DMA_Enable(UART7_Tx, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);

        // Can发送
        dataLength = Protocol_Pack(&UserChannel, id);
        Can_Send_Msg(CAN1, id, UserChannel.sendBuf, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        // DebugData.debug1 = ProtocolData.user.boardAlpha.data1 * 1000;
        // DebugData.debug2 = ProtocolData.user.boardBeta.data4 * 1000;
    }
    vTaskDelete(NULL);
}

void Task_Charge(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    PID_Init(&PID_Charge, 100, 0, 0, 3000, 0);

    float isSwitchClose;
    float targetSpeed;
    float motorSpeed;

    while (1) {
        motorSpeed    = Motor_Charge.speed / 19.2f;
        isSwitchClose = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);

        targetSpeed = 0;

        if (ChargeEnabled) {
            targetSpeed = -100;
        }

        if (isSwitchClose < 0.5) {
            targetSpeed = 0;
        }

        if (ChargeDisabled) {
            targetSpeed = 1000;
        }

        PID_Calculate(&PID_Charge, targetSpeed, motorSpeed);

        Can_Send(CAN1, 0x200, PID_Charge.output, 0, 0, 0);

        DebugData.debug1 = isSwitchClose;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Hook(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        PWM_Set_Compare(&PWM_Hook_L, HookClose ? 14 : 5);
        PWM_Set_Compare(&PWM_Hook_R, HookClose ? 19 : 100);
        vTaskDelayUntil(&LastWakeTime, intervalms);
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

    // 初始化全局变量
    Handle_Init();

    // 初始化硬件
    BSP_Init();

    // 调试任务
#if DEBUG_ENABLED
    // xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Debug_Gyroscope_Sampling, "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

    // 低级任务
    // xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 运动控制任务
    xTaskCreate(Task_Charge, "Task_Charge", 400, NULL, 6, NULL);
    xTaskCreate(Task_Hook, "Task_Hook", 400, NULL, 6, NULL);

    // 遥控器控制任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // DMA发送任务
    // xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Board_Communication, "Task_Board_Communication", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
