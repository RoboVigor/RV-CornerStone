#include "main.h"
#include "BSP_GPIO.h"
#include "BSP_NVIC.h"
#include "BSP_UART.h"
#include "BSP_DMA.h"
#include "BSP_CAN.h"
#include "BSP_TIM.h"

/**
 * @brief  初始化任务初始化任务和硬件
 * @param  void *Parameters
 * @return void
 */

void Task_Sys_Init(void *Parameters) {
    // 进入临界区
    taskENTER_CRITICAL();

    // BSP们
    BSP_GPIO_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_DMA_Init();
    BSP_TIM_Init();
    BSP_NVIC_Init();

    // 初始化陀螺仪
    MPU6500_IntConfiguration();
    MPU6500_Initialize();
    MPU6500_EnableInt();

    // 陀螺仪任务
    xTaskCreate(Task_Mpu6500, "Task_Mpu6500", 500, NULL, 6, &TaskHandle_Mpu6500);

    // 初始化消息体
    Queue_Test = xQueueCreate(10, sizeof(u32));

    // 建立事件标志组任务
    EventGroupHandler_YawAngleMode = xEventGroupCreate();
    xTaskCreate(Task_Event_Group, "Task_Event_Group", 500, NULL, 6, &TaskHandle_Event_Group);

    // 建立debug任务
    // xTaskCreate(Task_Debug, "Task_Debug", 500, NULL, 5, &TaskHandle_Debug);
    xTaskCreate(Task_MagicReceive, "Task_MagicReceive", 500, NULL, 6, NULL);
    xTaskCreate(Task_MagicSend, "Task_MagicSend", 500, NULL, 6, NULL);

    // 建立IRQ任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, &TaskHandle_Safe_Mode);
    xTaskCreate(Task_DBus, "Task_DBus", 400, NULL, 5, &TaskHandle_DBus);

    // 建立低优先级任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, &TaskHandle_Chassis);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, &TaskHandle_Blink);

    // 完成使命
    vTaskDelete(NULL);

    // 退出临界区
    taskEXIT_CRITICAL();
}
