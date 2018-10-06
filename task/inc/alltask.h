#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

// 系统初始化
void Task_Sys_Init(void *Parameters);

// 功能任务
void Task_Blink(void *Parameters);
void Task_Chassis(void *Parameters);
void Task_Sumsung(void *Parameters);
void Task_Mode_Switch(void *Parameters);
// void Task_Event_Group(void *pvParameter);
void Task_Safe_Mode(void *Parameters);

// 调试任务
void Task_RTOSState(void *Parameters);
void Task_MagicReceive(void *Parameters);
void Task_MagicSend(void *Parameters);

// 高频任务
void Task_Gyroscope(void *Parameters);

// 任务句柄
// __HANDLE_EXT TaskHandle_t TaskHandle_Blink;

#endif
