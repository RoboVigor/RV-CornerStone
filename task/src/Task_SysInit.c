#define __HANDLER_GLOBALS

#include "main.h"

/**
 * @brief  初始化任务初始化任务和硬件
 * @param  void *Parameters
 * @return void
 */

void Task_SysInit(void *Parameters) {
    // 进入临界区
    taskENTER_CRITICAL();

    // BSP们
    BSP_GPIO_InitConfig();
    BSP_CAN_InitConfig();
    BSP_UART_InitConfig();
    BSP_DMA_InitConfig();
    BSP_TIM_InitConfig();
    BSP_NVIC_InitConfig();

    // 初始化消息体
    queue_test = xQueueCreate(10, sizeof(u32));

    // 建立IRQ任务
    xTaskCreate(Task_Debug, "Task_Debug", 500, NULL, 6, &taskHandler_Debug);
    xTaskCreate(Task_DBUS, "Task_DBUS", 400, NULL, 6, &taskHandler_DBUS);
    vTaskSuspend(taskHandler_Debug);

    // 建立低优先级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, &taskHandler_Blink);

    // 完成使命
    vTaskDelete(NULL);

    // 退出临界区
    taskEXIT_CRITICAL();
}
