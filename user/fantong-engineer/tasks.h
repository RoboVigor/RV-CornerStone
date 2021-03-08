#ifndef __TASKS_H
#define __TASKS_H

void Task_Blink(void *Parameters);
void Task_Chassis(void *Parameters);
void Task_Fetch(void *Parameters);
void Task_Control(void *Parameters);
void Task_Board_Communication(void *Parameters);
void Task_Remote_Share(void *Parameters);
void Task_Gimbal(void *Parameters);
void Task_Can_Send(void *Parameters);
void Task_Milk(void *Parameters);
void Task_Go(void *Parameters);
void Task_Raise(void *Parameter);
void Task_Rescue(void *Parameter);
void Task_Startup_Music(void *Parameters);
void Task_Fire_Stir(void *Parameters);
void Task_Snail(void *Parameters);

#endif
