#include "Driver_Magic.h"
#include "Driver_Filter.h"
#include "macro.h"

void Raise_Error(Node_Type *node, uint16_t code, char *text) {
    node->protocolData->errorInfo.code = code;
    strcpy(node->protocolData->errorInfo.text, text);
    Bridge_Send_Protocol_Once(node, 0x6666);
}

void Send_Debug_Info(Node_Type *node, DebugData_Type debugData) {
    node->protocolData->debugInfo.debugData = debugData;
    Bridge_Send_Protocol_Once(node, 0x1024);
}

void Clibrate_Gyroscope() {
    TickType_t  LastWakeTime      = xTaskGetTickCount();
    Filter_Type Filter_Sampling   = {.count = 0};
    int32_t     totalSampleNumber = 3000;
    int16_t     printEvery        = 100;
    Filter_Update(&Filter_Sampling, ABS(Gyroscope_Get_Filter_Diff()));
    Filter_Update_Sample(&Filter_Sampling);
    if (Filter_Sampling.count % printEvery == 0) {
        printf("#%d\r\n average: %f\r\n max: %f\r\n min: %f\r\n\r\n",
               Filter_Sampling.count,
               Filter_Sampling.movingAverage,
               Filter_Sampling.max,
               Filter_Sampling.min);
    }
    if (Filter_Sampling.count == totalSampleNumber) {
        return;
    }
}

void Inspect_RTOS() {
    extern volatile uint32_t ulHighFrequencyTimerTicks;
    int                      taskDebug_Sign = 0;
    u8                       pcWriteBuffer[1000];
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
}