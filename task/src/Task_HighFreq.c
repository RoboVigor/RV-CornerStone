/**
 * @brief 高频任务
 */

#include "main.h"

void Task_Gyroscope(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟

    while (1) {
        Gyroscope_Update_Angle_Data();
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}