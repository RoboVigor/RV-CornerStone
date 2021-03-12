#ifndef __TASKS_H
#define __TASKS_H

void Task_Safe_Mode(void *Parameters);
void Task_Control(void *Parameters);
void Task_Chassis(void *Parameters);
void Task_Up_Gimbal(void *Parameters);
void Task_Down_Gimbal(void *Parameters);
void Task_Up_Stir(void *Parameters);
void Task_Down_Stir(void *Parameters);
void Task_Up_Frict(void *Parameters);
void Task_Down_Frict(void *Parameters);
void Task_Can_Send(void *Parameters);
void Task_Board_Communication(void *Parameters);
void Task_Blink(void *Parameters);
void Task_Startup_Music(void *Parameters);

#endif
