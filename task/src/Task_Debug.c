/**
 * @brief 调试任务
 */

#include "main.h"

/**
 * @brief  调试信息输出任务
 */

extern volatile uint32_t ulHighFrequencyTimerTicks;
int                      taskDebug_Sign = 0;

void Task_RTOSState(void *Parameters) {
    while (1) {
        u8 pcWriteBuffer[1000];
        printf("=========================\r\n");
        printf("time:    %d\r\n", ulHighFrequencyTimerTicks);
        if (taskDebug_Sign) {
            printf("\r\nName            Count              Usage\r\n");
            vTaskGetRunTimeStats((char *) &pcWriteBuffer);
            printf("%s\r\n", pcWriteBuffer);
        } else {
            printf("Name         Status   Priority  RemainStackSize   Number\r\n");
            vTaskList((char *) &pcWriteBuffer);
            printf("%s\r\n", pcWriteBuffer);
        }
        taskDebug_Sign = taskDebug_Sign ? 0 : 1;

        printf("=========================\r\n\r\n\r\n");
        vTaskDelay(5000);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 地面站 串口调试数据 接收函数
 */
void Task_MagicReceive(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    Magic_Init_Handle(&magic, 0); // 初始化调试数据的默认值
    while (1) {
        taskENTER_CRITICAL();          // 进入临界段
        Magic_Get_Debug_Value(&magic); // 接收调试数据
        taskEXIT_CRITICAL();           // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 地面站 反馈数据 发送函数
 */
void Task_MagicSend(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    float      rpm2rps      = 3.14 / 60; // 转子的转速(round/min)换算成角速度(rad/s)

    while (1) {
        taskENTER_CRITICAL(); // 进入临界段
        // printf("%10f %10f %10f %10f\r\n", Motor_LF.angle, Motor_LB.angle, Motor_RB.angle, Motor_RF.angle);
        printf("%.2f %.2f %.2f\r\n",
               Motor_LeftFrict.speed * rpm2rps,
               -Motor_RightFrict.speed * rpm2rps,
               Motor_LeftFrict.speed * rpm2rps + Motor_LeftFrict.speed * rpm2rps);
        taskEXIT_CRITICAL(); // 退出临界段
        vTaskDelayUntil(&LastWakeTime, 500);
    }
    vTaskDelete(NULL);
}
