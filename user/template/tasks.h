#ifndef __TASKS_H
#define __TASKS_H

void Task_Control(void *Parameters);
void Task_Blink(void *Parameters);
void Task_Chassis(void *Parameters);
void Task_Host(void *Parameters);
void Task_Can_Send(void *Parameters);
void Task_Startup_Music(void *Parameters);
void Task_OLED(void *Parameters);

#endif
