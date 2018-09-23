#include "main.h"

int debug1 = 0;
int debug2 = 0;
int debug3 = 0;
/**
 * @brief  DBus处理任务
 * @param  void *Parameters
 * @return void
 * 暂时作消息体测试
 */

void Task_DBus(void *Parameters) {
    u32 data;

    while (1) {
        if (xQueueReceive(Queue_Test, &data, portMAX_DELAY)) {
            printf("Received message: %d\r\n", data);
        } else {
            printf("Failed");
        }
    }

    vTaskDelete(NULL);
}

/**
 * @brief  调试信息输出任务
 * @param  void *Parameters
 * @return void
 * @todo   无线串口考虑改为定点传输
 * @todo   不在调试模式的时候挂起该任务
 * @todo   当然前提是我们有调试模式了
 */

extern volatile uint32_t ulHighFrequencyTimerTicks;
int                      taskDebug_Sign = 0;

void Task_Debug(void *Parameters) {
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
 *
 * @param Parameters
 */
void Task_MagicReceive(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    Magic_Init_Handle(&magic, 0); // 初始化调试数据的默认值
    while (1) {
        taskENTER_CRITICAL();          // 进入临界段代码（在不进入的情况下有被抢占的情况）
        Magic_Get_Debug_Value(&magic); // 接收调试数据
        taskEXIT_CRITICAL();           // 退出临界段代码
        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 地面站 反馈数据 发送函数
 *
 * @param Parameters
 */
void Task_MagicSend(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        u8 i = 0;
        MIAO(i, 1, 2);
        taskENTER_CRITICAL(); // 进入临界段代码（在不进入的情况下有被抢占的情况）
        // 发送反馈数据
        // ChassisAnglePID1
        // CM1PID
        printf("MGC %d \r\n", magic.value);
        // printf("ESR: %x\r\n", CAN2->ESR);
        // printf("FEED %f \r\n\r\n", ChassisAnglePID1.feedback);
        printf("OUT  %f %f %d \r\n", ChassisAnglePID1.feedback, ChassisAnglePID1.target, ChassisAnglePID1.output);
        // printf("OUT %f \r\n", ChassisAnglePID1.feedback - ChassisAnglePID1.target);
        printf("IN  %f %f %d \r\n", CM1PID.feedback, CM1PID.target, CM1PID.output);
        // printf("ENC %d %d %d %f\r\n", CM1_Encoder.rawValue, CM1_Encoder.ecdBias, CM1_Encoder.roundCnt, CM1_Encoder.ecdAngle);
        // printf("SPEED %d \r\n", Motor_Feedback.motor201Speed);
        printf("---------------------\r\n");

        taskEXIT_CRITICAL(); // 退出临界段代码
        vTaskDelayUntil(&LastWakeTime, 500);
    }
    vTaskDelete(NULL);
}

/**
 * @brief  安全模式
 * @param  void *Parameters
 * @return void
 */
void Task_Safe_Mode(void *Parameters) {

    while (1) {
        if (DBusData.switchRight == 2) {
            Can_Set_CM_Current(CAN1, 0, 0, 0, 0);
            vTaskSuspendAll();
        }
        vTaskDelay(100);
    }

    vTaskDelete(NULL);
}

void Task_Update(void *parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();
    while (1) {
        CAN_Update_Encoder_Data(&CM1_Encoder, Motor_Feedback.motor201Angle);
        debug1 = CM1_Encoder.ecdAngle;
        debug2 = CM1_Encoder.rawValue;
        debug3 = CM1_Encoder.diff;
        vTaskDelayUntil(&LastWakeTime, 10);
    }
    vTaskDelete(NULL);
}
