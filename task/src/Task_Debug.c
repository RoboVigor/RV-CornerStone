/**
 * @brief 调试任务
 */

#include "main.h"

/**
 * @brief  调试信息输出任务
 */

extern volatile uint32_t ulHighFrequencyTimerTicks;
int                      taskDebug_Sign = 0;

void Task_RTOSState(void *Parameters) {
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

/**
 * @brief 地面站 串口调试数据 接收函数
 */
void Task_MagicReceive(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    Magic_Init_Handle(&magic, 2); // 初始化调试数据的默认值
    while (1) {
        taskENTER_CRITICAL();          // 进入临界段代码（在不进入的情况下有被抢占的情况）
        Magic_Get_Debug_Value(&magic); // 接收调试数据
        taskEXIT_CRITICAL();           // 退出临界段代码
        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 地面站 反馈数据 发送函数
 */
void Task_MagicSend(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        u8 i = 0;
        //        MIAO(i, 1, 2);
        taskENTER_CRITICAL(); // 进入临界段代码（在不进入的情况下有被抢占的情况）
        // 发送反馈数据
        // printf("p:%f \r\n", PID_LFCM.p);
        // printf("error: %f \r\n", PID_LFCM.error);
        // printf("target: %f \r\n", PID_LFCM.target);
        // printf("feedback: %f \r\n", PID_LFCM.feedback);
        printf("magic.Value: %d \r\n", magic.value);
        // printf("%d %d %f %d\r\n", magic.value, PID_LFCM.error, PID_LFCM.output, i);
        taskEXIT_CRITICAL(); // 退出临界段代码
        vTaskDelayUntil(&LastWakeTime, 3000);
    }
    vTaskDelete(NULL);
}
