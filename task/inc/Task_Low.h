#ifndef __TASK_LOW_H
#define __TASK_LOW_H

void Task_Blink(void *Parameters);
void Task_Chassis(void *Parameters);
void Task_Wheel(void *Parameters);

void _Set_CM_Current(int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);

#endif
