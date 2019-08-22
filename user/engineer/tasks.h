#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

#if DEBUG_ENABLED
void Task_Debug_Magic_Send(void *Parameters);         // 无线串口调试输出
void Task_Debug_Magic_Receive(void *Parameters);      // 无线串口调试接收
void Task_Debug_RTOS_State(void *Parameters);         // 输出RTOS系统资源
void Task_Debug_Gyroscope_Sampling(void *Parameters); // 采样任务
#endif

// Base
void Task_Sys_Init(void *Parameters);      // 硬件初始化和任务初始化
void Task_Blink(void *Parameters);         // 确认存活
void Task_Safe_Mode(void *Parameters);     // 安全模式
void Task_Startup_Music(void *Parameters); // 开机音乐

// Sturcture
void Task_Chassis(void *Parameters);
void Task_Distance_Sensor(void *Parameters);
void Task_Board_Communication_Send(void *Parameters);
void Task_Board_Communication_Recive(void *Parameters);
void Task_Take_Fsm(void *Parameters);
void Task_Supply(void *Parameters);
void Task_Rescue(void *Parameters);
void Task_Visual(void *Parameters);
void Task_Optoelectronic_Input_Take(void *Parameters);
void Task_Upthrow_Horizontial(void *Parameters);
void Task_Take_Rotate(void *Parameters);
void Task_Take_Vertical_GuideWheel(void *Parameters);
void Task_Landing(void *Parameters);
void Task_Upthrow(void *Parameter);
void Task_Limit_Switch(void *Parameters);

#endif
