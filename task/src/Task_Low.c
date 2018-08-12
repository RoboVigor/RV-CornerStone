#include "main.h"

/**
 * @brief  LED闪烁任务 确认存活
 * @param  void *Parameters
 * @return void
 */

void Task_Blink(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        GREEN_LIGHT_TOGGLE;
        vTaskDelayUntil(&LastWakeTime, 250);
    }

    vTaskDelete(NULL);
}
