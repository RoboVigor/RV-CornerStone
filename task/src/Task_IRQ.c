#include "main.h"

void Task_DBUS(void *Parameters) {
  while (1) {
    //RED_LIGHT_TOGGLE;
    delay_ms(1000);
  }
  // while (1) {
  //   uint8_t UARTtemp;
  //   UARTtemp = USART1->DR;
  //   UARTtemp = USART1->SR;

  //   //RED_LIGHT_TOGGLE;
  //   //delay_ms(1000);

  //   DMA_Cmd(DMA2_Stream2, DISABLE);

  //   //数据量正确
  //   if (DMA2_Stream2->NDTR == DBUSBackLength) {
  //     DBUS_DataDecoding(); //解码
  //   }

  //   //重启DMA
  //   DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
  //   while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
  //   DMA_SetCurrDataCounter(DMA2_Stream2, DBUSLength + DBUSBackLength);
  //   DMA_Cmd(DMA2_Stream2, ENABLE);
  // }

  vTaskDelete(NULL);
}

void Task_USART3(void *Parameters) {
  while (1) {
    printf("a");
    delay_us(500 * 1000);
  }

  vTaskDelete(NULL);
}
