#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

#if DEBUG_ENABLED
void Task_Debug_RTOS_State(void *Parameters);         // 输出RTOS系统资源
void Task_Debug_Gyroscope_Sampling(void *Parameters); // 采样任务
#endif

void Task_Sys_Init(void *Parameters);            // 硬件初始化和任务初始化
void Task_OLED(void *Parameters);                // oled屏幕
void Task_Blink(void *Parameters);               // led动画
void Task_Startup_Music(void *Parameters);       // 开机音乐
void Task_Duct(void *Parameters);                // 涵道
void Task_Servo(void *Parameters);               // 舵机
void Task_Board_Communication(void *Parameters); // 板间通讯

#endif
