/**
 * @brief 高频任务
 */

#include "main.h"

void Task_Gyroscope(void *Parameters) {
  TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
  cloud_counter = 0;
  while (1) {
    if (GYROSCOPE_YAW_QUATERNIONABSTRACTION == 1) {
      if (cloud_counter <= COUNT_QUATERNIONABSTRACTION) {
        cloud_counter = cloud_counter + 1;
      }
    }
    Gyroscope_Update_Angle_Data();

    vTaskDelayUntil(&LastWakeTime, 3);
  }
  vTaskDelete(NULL);
}
