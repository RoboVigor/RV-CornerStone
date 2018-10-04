#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

// 系统初始化
void Task_Sys_Init(void *Parameters); // 初始化任务初始化任务和硬件

// 功能任务
void Task_Blink(void *Parameters);     // 确认存活
void Task_Chassis(void *Parameters);   // 底盘运动
void Task_Safe_Mode(void *Parameters); // 安全模式

// 调试任务
void Task_RTOSState(void *Parameters);    // 输出RTOS系统资源
void Task_MagicReceive(void *Parameters); // 无线串口调试接收
void Task_MagicSend(void *Parameters);    // 无线串口调试输出

// 高频任务
void Task_Gyroscope(void *Parameters); // 陀螺仪姿态解算

#endif
