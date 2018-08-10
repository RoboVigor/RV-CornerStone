#include "main.h"

void Task_Blink(void *Parameters) {
  int sign = 0;

  while (1) {
    if (sign) {
      RED_LIGHT_ON;
    } else {
      RED_LIGHT_OFF;
    }
    sign = sign ? 0 : 1;
    vTaskDelay(250);
  }

  vTaskDelete(NULL);
}
