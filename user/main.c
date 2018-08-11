#include "main.h"

int main(void) {
  BSP_GPIO_InitConfig();
  BSP_CAN_InitConfig();
  BSP_UART_InitConfig();
  BSP_DMA_InitConfig();
  BSP_TIM_InitConfig();
  BSP_NVIC_InitConfig();

  delay_init(180); // 延时初始化
  uart_init(9600); // 初始化串口
  LED_Init();      // 初始化LED
  BEEP_Init();     // 初始化蜂鸣器

  //创建系统初始化任务
  xTaskCreate(Task_SysInit, "Task_SysInit", 1600, NULL, 1, NULL);

  //启动调度，开始执行任务
  vTaskStartScheduler();

  //系统启动失败：定时器任务或者空闲任务的heap空间不足
  while (1);
}
