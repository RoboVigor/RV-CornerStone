#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

void Task_Blink(void *Parameters);         // 确认存活
void Task_Control(void *Parameters);       // 手柄和键盘的控制
void Task_Communication(void *Parameters); // 协议通讯
void Task_Hook(void *Parameters);          // 载飞镖架的固定与释放
void Task_Charge(void *Parameters);        // 飞镖架的蓄能
void Task_Safe_Mode(void *Parameters);     // 安全模式
void Task_Startup_Music(void *Parameters); // 开机音乐
void Task_Can_Send(void *Parameters);      // CAN发送任务

#endif
