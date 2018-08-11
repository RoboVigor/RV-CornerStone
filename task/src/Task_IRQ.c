#include "main.h"

int debug_j = 0;

void Task_DBUS(void *Parameters) {
  //uint8_t UARTtemp;
  while (1) {
    debug_j += 1;
    delay_ms(1000);
    //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // RED_LIGHT_ON;
    // UARTtemp = USART1->DR;
    // UARTtemp = USART1->SR;

    // DMA_Cmd(DMA2_Stream2, DISABLE);

    // //数据量正确
    // if (DMA2_Stream2->NDTR == DBUSBackLength) {
    //   DBUS_DataDecoding(); //解码
    // }

    // //重启DMA
    // DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
    // while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
    // DMA_SetCurrDataCounter(DMA2_Stream2, DBUSLength + DBUSBackLength);
    // DMA_Cmd(DMA2_Stream2, ENABLE);
  }

  vTaskDelete(NULL);
}

void Task_USART3(void *Parameters) {
  while (1) {
    printf("1");
    vTaskDelay(5000);
  }

  vTaskDelete(NULL);
}
