#ifndef __TASK_IRQ_H
#define __TASK_IRQ_H

void Task_DBus(void *Parameters);
void Task_Debug(void *Parameters);
void Task_MagicReceive(void *Parameters);
void Task_MagicSend(void *Parameters);
void Task_Safe_Mode(void *Parameters);
void Task_Update(void *Parameters);

#endif
