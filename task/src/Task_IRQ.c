#include "main.h"

/**
 * @brief  DBUS处理任务
 * @param  void *Parameters
 * @return void
 * 处理DBUS消息
 * 暂时作测试（红灯闪烁）
 */

void Task_DBUS(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        RED_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
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
int                      Task_Debug_Sign = 0;

void Task_Debug(void *Parameters) {
    while (1) {
        u8 pcWriteBuffer[ 1000 ];
        printf("=========================\r\n");
        printf("time:    %d\r\n", ulHighFrequencyTimerTicks);
        if (Task_Debug_Sign) {
            printf("\r\nName            Count              Usage\r\n");
            vTaskGetRunTimeStats((char *) &pcWriteBuffer);
            printf("%s\r\n", pcWriteBuffer);
        } else {
            printf("Name         Status   Priority  RemainStackSize   Number\r\n");
            vTaskList((char *) &pcWriteBuffer);
            printf("%s\r\n", pcWriteBuffer);
        }
        Task_Debug_Sign = Task_Debug_Sign ? 0 : 1;

        printf("=========================\r\n\r\n\r\n");
        vTaskDelay(5000);
    }
    vTaskDelete(NULL);
}
