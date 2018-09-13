#include "main.h"

/**
 * @brief  DBus处理任务
 * @param  void *Parameters
 * @return void
 * 暂时作消息体测试
 */

void Task_DBus(void *Parameters) {
    u32 data;

    while (1) {
        if (xQueueReceive(Queue_Test, &data, portMAX_DELAY)) {
            printf("Received message: %d\r\n", data);
        } else {
            printf("Failed");
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief  调试信息输出任务
 * @param  void *Parameters
 * @return void
 * @todo   无线串口考虑改为定点传输
 * @todo   不在调试模式的时候挂起该任务
 * @todo   当然前提是我们有调试模式了
 */

extern volatile uint32_t ulHighFrequencyTimerTicks;
int                      taskDebug_Sign = 0;

void Task_Debug(void *Parameters) {
    while (1) {
        u8 pcWriteBuffer[1000];
        printf("=========================\r\n");
        printf("time:    %d\r\n", ulHighFrequencyTimerTicks);
        if (taskDebug_Sign) {
            printf("\r\nName            Count              Usage\r\n");
            vTaskGetRunTimeStats((char *) &pcWriteBuffer);
            printf("%s\r\n", pcWriteBuffer);
        } else {
            printf("Name         Status   Priority  RemainStackSize   Number\r\n");
            vTaskList((char *) &pcWriteBuffer);
            printf("%s\r\n", pcWriteBuffer);
        }
        taskDebug_Sign = taskDebug_Sign ? 0 : 1;

        printf("=========================\r\n\r\n\r\n");
        vTaskDelay(5000);
    }
    vTaskDelete(NULL);
}

void Task_MagicReceive(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    USART_Set_Default_Debug_Number(0);
    while (1) {
        taskENTER_CRITICAL();
        MagicNumber = USART_Get_Debug_Number();
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

void Task_MagicSend(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        u8 i = 0;
        MIAO(i, 1, 2);
        taskENTER_CRITICAL();
        printf("----- \r\n");
        // printf("p:%f \r\n", CM1PID.p);
        // printf("error: %f \r\n", CM1PID.error);
        // printf("target: %f \r\n", CM1PID.target);
        printf("feedback: %f \r\n", CM1PID.feedback);
        printf("output: %d \r\n", MagicNumber);
        // printf("%d %d %f %d\r\n", MagicNumber, CM1PID.error, CM1PID.output, i);
        taskEXIT_CRITICAL();
        vTaskDelayUntil(&LastWakeTime, 3000);
    }
    vTaskDelete(NULL);
}

/**
 * @brief  安全模式
 * @param  void *Parameters
 * @return void
 */

void Task_Safe_Mode(void *Parameters) {

    while (1) {
        if (DBusData.switchRight == 2) {
            Can_Set_CM_Current(CAN1, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }

    vTaskDelete(NULL);
}
