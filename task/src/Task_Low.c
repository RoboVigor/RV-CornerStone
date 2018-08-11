#include "main.h"

void Task_Blink(void *Parameters) {
  int sign = 0;
  TickType_t LastWakeTime = xtaskgettic;
  while (1) {
    if (sign) {
      GREEN_LIGHT_ON;
    } else {
      GREEN_LIGHT_OFF;
    }
    sign = sign ? 0 : 1;
    vTaskDelayUntil(&LastWakeTime, 50);
  }

  vTaskDelete(NULL);
}
