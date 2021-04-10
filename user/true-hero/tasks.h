#ifndef __TASKS_H
#define __TASKS_H

void Task_Control(void *Parameters);
void Task_Blink(void *Parameters);
void Task_Chassis(void *Parameters);
// void Task_Communication(void *Parameters);
void Task_Can_Send(void *Parameters);
void Task_Startup_Music(void *Parameters);
void Task_OLED(void *Parameters);
void Task_Debug_Magic_Send(void *Parameters);
void Task_Gimbal(void *Parameters);
void Task_Fire(void *Parameters);
void Task_Control(void *Parameters);
void Task_Client_Communication(void *Parameters);
void Task_Sys_Init(void *Parameters);

#endif
