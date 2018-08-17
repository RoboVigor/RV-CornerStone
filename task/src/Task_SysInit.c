#define __HANDLER_GLOBALS

#include "main.h"

/**
 * @brief  初始化任务初始化任务和硬件
 * @param  void *Parameters
 * @return void
 */

void Task_SysInit(void *Parameters) {
    // BSP们
    BSP_GPIO_InitConfig();
    BSP_CAN_InitConfig();
    BSP_UART_InitConfig();
    BSP_DMA_InitConfig();
    BSP_TIM_InitConfig();
    BSP_NVIC_InitConfig();

    // 建立任务
    //// IRQ任务
    //xTaskCreate(Task_Debug, "Task_Debug", 500, NULL, 6, &TaskHandler_Debug);
    //xTaskCreate(Task_DBUS, "Task_DBUS", 400, NULL, 6, &TaskHandler_DBUS);
    //// 低优先级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, &TaskHandler_Blink);

    printf("4");

    // 完成使命
    vTaskDelete(NULL);
}
