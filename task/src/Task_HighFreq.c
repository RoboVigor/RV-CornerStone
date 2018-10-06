/**
 * @brief 高频任务
 */

#include "main.h"

/**
 * @brief 陀螺仪姿态解算
 */
void Task_Gyroscope(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟

    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        Gyroscope_Update_Angle_Data();
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 5);
    }
    vTaskDelete(NULL);
}