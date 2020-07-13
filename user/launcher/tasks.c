/**
 * @brief 甩锅小车
 * @version 1.2.0
 */
#include "main.h"

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

    PID_Init(&PID_Charge, 60, 5, 0, 16000, 2000);

    float isSwitchClose;
    float targetSpeed;
    float motorSpeed;

    while (1) {
        motorSpeed    = Motor_Charge.speed / 19.2f;
        isSwitchClose = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_5);

        targetSpeed = 0;

        if (ChargeEnabled) {
            targetSpeed = -150;
        }

        if (isSwitchClose < 0.5) {
            targetSpeed = 0;
        }

        if (ChargeDisabled) {
            targetSpeed = 200;
        }

        PID_Calculate(&PID_Charge, targetSpeed, motorSpeed);

        Motor_Charge.input = PID_Charge.output;

        DebugData.debug1 = isSwitchClose;
        DebugData.debug2 = PID_Charge.output;
        DebugData.debug3 = Motor_Charge.speed;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Hook(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        PWM_Set_Compare(&PWM_Hook_L, HookClose ? 19 : 25);
        PWM_Set_Compare(&PWM_Hook_R, HookClose ? 15 : 9);
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

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 6, NULL);

    // 完成使命
    vTaskDelete(NULL);
}
