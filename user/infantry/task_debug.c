/**
 * @brief 调试任务
 */

#include "main.h"

void Task_Debug_RTOS_State(void *Parameters) {
    extern volatile uint32_t ulHighFrequencyTimerTicks;
    int                      taskDebug_Sign = 0;
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

void Task_Debug_Magic_Receive(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        taskENTER_CRITICAL();          // 进入临界段
        Magic_Get_Debug_Value(&magic); // 接收调试数据
        taskEXIT_CRITICAL();           // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

void Task_Debug_Gyroscope_Sampling(void *Parameters) {
    TickType_t  LastWakeTime      = xTaskGetTickCount();
    Filter_Type Filter_Sampling   = {.count = 0};
    int32_t     totalSampleNumber = 3000;
    int16_t     printEvery        = 100;

    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        Filter_Update(&Filter_Sampling, ABS(Gyroscope_Get_Filter_Diff()));
        Filter_Update_Sample(&Filter_Sampling);
        if (Filter_Sampling.count % printEvery == 0) {
            printf("#%d\r\n average: %f\r\n max: %f\r\n min: %f\r\n\r\n",
                   Filter_Sampling.count,
                   Filter_Sampling.movingAverage,
                   Filter_Sampling.max,
                   Filter_Sampling.min);
        }
        if (Filter_Sampling.count == totalSampleNumber) {
            break;
        }
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}
