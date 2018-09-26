#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

void Task_Sys_Init(void *Parameters);

void Task_Blink(void *Parameters);
void Task_Chassis(void *Parameters);
void Task_Event_Group(void *pvParameter);
void Task_Mpu6500(void *Parameters);

void Task_DBus(void *Parameters);
void Task_Debug(void *Parameters);
void Task_MagicReceive(void *Parameters);
void Task_MagicSend(void *Parameters);
void Task_Safe_Mode(void *Parameters);

#endif
