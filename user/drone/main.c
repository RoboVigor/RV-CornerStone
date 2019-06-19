#define __HANDLE_GLOBALS
#include "main.h"

int main(void) {

  //设置中断优先级位数
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  //调试相关
  Delay_Init(180); // 初始化延时
  LED_Init();      // 初始化LED
  Beep_Init();     // 初始化蜂鸣器

  //创建系统初始化任务
  xTaskCreate(Task_Sys_Init, "Task_Sys_Init", 400, NULL, 1, NULL);

  //启动调度,开始执行任务
  vTaskStartScheduler();

  //系统启动失败:定时器任务或者空闲任务的heap空间不足
  while (1) {
  }
}
