#include "main.h"
#include "handle.h"
#include "BSP_GPIO.h"
#include "BSP_NVIC.h"
#include "BSP_UART.h"
#include "BSP_DMA.h"
#include "BSP_CAN.h"
#include "BSP_TIM.h"

/**
 * @brief  初始化任务初始化任务和硬件
 */

void Task_Sys_Init(void *Parameters) {
    // 进入临界区
    taskENTER_CRITICAL();

    // 初始化全局变量
    Handle_Init();

    // BSP们
    BSP_GPIO_Init();
    BSP_CAN_Init();
    BSP_UART_Init();
    BSP_DMA_Init();
    BSP_TIM_Init();
    BSP_NVIC_Init();

    // 初始化陀螺仪
    // MPU6500_IntConfiguration();
    // MPU6500_Initialize();
    // MPU6500_EnableInt();

    // 调试任务
    // xTaskCreate(Task_RTOSState, "Task_RTOSState", 500, NULL, 6, NULL);
    xTaskCreate(Task_MagicReceive, "Task_MagicReceive", 500, NULL, 6, NULL);
    xTaskCreate(Task_MagicSend, "Task_MagicSend", 500, NULL, 6, NULL);

    // 高频任务
    // xTaskCreate(Task_Gyroscope, "Task_Gyroscope", 400, NULL, 5, NULL);

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);
    xTaskCreate(Task_Fire, "Task_Fire", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Event_Group, "Task_Event_Group", 400, NULL, 3, NULL);

    // 完成使命
    vTaskDelete(NULL);

    // 退出临界区
    taskEXIT_CRITICAL();
}
