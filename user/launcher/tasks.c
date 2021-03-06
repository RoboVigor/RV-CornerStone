/**
 * @brief 甩锅小车
 * @version 1.2.0
 */

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "tasks.h"

void Task_Control(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        ChargeMode  = RIGHT_SWITCH_TOP;
        ReleaseMode = RIGHT_SWITCH_BOTTOM;
        HookMode    = LEFT_SWITCH_TOP;
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}

void Task_Communication(void *Parameters) {
    TickType_t         LastWakeTime = xTaskGetTickCount(); // 时钟
    float              interval     = 0.1;                 // 任务运行间隔 s
    int                intervalms   = interval * 1000;     // 任务运行间隔 ms
    ProtocolInfo_Type *protocolInfo = Protocol_Get_Info_Handle(0x501);

    while (1) {

        // 修改数据
        ProtocolData.boardAlpha.data1 = 1;
        ProtocolData.boardAlpha.data2 = 2;
        ProtocolData.boardAlpha.data3 = 3;
        ProtocolData.boardAlpha.data4 = 4;
        Bridge_Send_Protocol_Once(&Node_Host, 0x501);

        // 发送频率
        vTaskDelayUntil(&LastWakeTime, intervalms);

        // 调试信息
        DebugData.debug1 = ProtocolData.boardAlpha.data1 * 1000;
        DebugData.debug2 = ProtocolData.boardAlpha.data2 * 1000;
        DebugData.debug3 = Node_Host.sendSeq;
        DebugData.debug4 = protocolInfo->receiveCount;
    }
    vTaskDelete(NULL);
}

void Task_Charge(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    PID_Init(&PID_Charge, 60, 5, 0, 16000, 4000);

    uint8_t switchClose;

    float targetSpeed;
    float motorSpeed;

    while (1) {
        motorSpeed  = Motor_Charge.speed / 19.2f;
        switchClose = !GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_5);

        if (ChargeMode && !switchClose) {
            targetSpeed = -150;
        } else if (ReleaseMode) {
            targetSpeed = 50;
        } else {
            targetSpeed = 0;
        }

        PID_Calculate(&PID_Charge, targetSpeed, motorSpeed);

        Motor_Charge.input = PID_Charge.output;

        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Hook(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.05;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        PWM_Set_Compare(&PWM_Hook_L, HookMode ? 17 : 25);
        PWM_Set_Compare(&PWM_Hook_R, HookMode ? 17 : 9);
        vTaskDelayUntil(&LastWakeTime, intervalms);
    }
    vTaskDelete(NULL);
}

void Task_Can_Send(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
    float      interval     = 0.01;                // 任务运行间隔 s
    int        intervalms   = interval * 1000;     // 任务运行间隔 ms

    while (1) {
        Bridge_Send_Motor(&BridgeData, SafetyMode);
        vTaskDelayUntil(&LastWakeTime, intervalms); // 发送频率
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