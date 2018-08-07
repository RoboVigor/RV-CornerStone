#include "main.h"

void Task_SysInitConfig(void *Parameters)
{
    // 测试任务
    int sign = 0;
    while (1)
    {
        if (sign)
        {
            RED_LIGHT_ON;
            GREEN_LIGHT_OFF;
        }
        else
        {
            RED_LIGHT_OFF;
            GREEN_LIGHT_ON;
        }
        sign = sign ? 0 : 1;
    }

    vTaskDelete(NULL);
}
