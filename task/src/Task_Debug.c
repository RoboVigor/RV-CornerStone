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

    Magic_Init_Handle(&magic, 700); // 初始化调试数据的默认值
    while (1) {
        Magic_Get_Debug_Value(&magic); // 接收调试数据
        vTaskDelayUntil(&LastWakeTime, 50);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 地面站 反馈数据 发送函数
 */
extern int targetBackGroupOffset;
extern int isTurning;
extern int targetFrontGroupOffset;
extern int lastSumsungMode;
extern int modeChangedAuto;
void       Task_MagicSend(void *Parameters) {
    TickType_t LastWakeTime = xTaskGetTickCount();

    while (1) {
        // printf("Mode %d Magic %d Angle %f %f %f %f\r\n",
        //        sumsungMode,
        //        magic.value,
        //        Motor_SumsungLF.angle,
        //        Motor_SumsungRF.angle,
        //        Motor_SumsungLB.angle,
        //        Motor_SumsungRB.angle);
        // printf("Mode %d Magic %d Error %f %f %f %f\r\n",
        //        sumsungMode,
        //        magic.value,
        //        ChassisAnglePID1.error,
        //        ChassisAnglePID4.error,
        //        ChassisAnglePID2.error,
        //        ChassisAnglePID3.error);
        printf("Pitch %f ax %d ay %d mode %d lmode %d autoc %d\r\n",
               EulerAngle.Roll,
               mpu6500_data.gx,
               mpu6500_data.gy,
               sumsungMode,
               lastSumsungMode,
               modeChangedAuto);
        // printf("Mode %d Magic %d Yaw %f turn %d\r\n", sumsungMode, magic.value, EulerAngle.Yaw, isTurning);
        // PID_Print(&PID_LFCM);
        vTaskDelayUntil(&LastWakeTime, 1000);
    }
    vTaskDelete(NULL);
}
