#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

#if DEBUG_ENABLED
void Task_Debug_Magic_Send(void *Parameters);    // 无线串口调试输出
void Task_Debug_Magic_Receive(void *Parameters); // 无线串口调试接收
void Task_Debug_RTOS_State(void *Parameters);    // 输出RTOS系统资源
void Task_Debug_Gyroscope_Sampling(void *Parameters); // 采样任务
#endif

void Task_Sys_Init(void *Parameters);      // 硬件初始化和任务初始化
void Task_Blink(void *Parameters);         // 确认存活
void Task_Startup_Music(void *parameters); //启动音乐
void Task_Chassis(void *Parameters);       // 底盘运动
void Task_Safe_Mode(void *Parameters);     // 安全模式
void Task_Gimbal(void *Parameters);        // 云台
void Task_Snail(void *Parameters);         // snail电机
void Task_Fire(void *Parameters);          // 拨弹轮
void Task_Safe_Mode(void *Parameters);     // safemode

#endif
