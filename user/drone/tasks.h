#ifndef __TASKS_H
#define __TASKS_H

#if DEBUG_ENABLED
void Task_Debug_Magic_Send(void *Parameters); // 无线串口调试输出
#endif

void Task_Sys_Init(void *Parameters);      // 硬件初始化和任务初始化
void Task_Blink(void *Parameters);         // 确认存活
void Task_Startup_Music(void *parameters); //启动音乐
void Task_Safe_Mode(void *Parameters);     // 安全模式
void Task_Gimbal(void *Parameters);        // 云台
void Task_Snail(void *Parameters);         // snail电机
void Task_Fire(void *Parameters);          // 拨弹轮
void Task_Safe_Mode(void *Parameters);     // safemode
void Task_Control(void *Parameters);       // 遥控器模式
void Task_Can_Send(void *Parameters);
#endif
