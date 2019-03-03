#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

#include "BSP_GPIO.h"
#include "BSP_NVIC.h"
#include "BSP_UART.h"
#include "BSP_DMA.h"
#include "BSP_CAN.h"
#include "BSP_TIM.h"

#if DEBUG_ENABLED
void Task_Debug_Magic_Send(void *Parameters);         // 无线串口调试输出
void Task_Debug_Magic_Receive(void *Parameters);      // 无线串口调试接收
void Task_Debug_RTOS_State(void *Parameters);         // 输出RTOS系统资源
void Task_Debug_Gyroscope_Sampling(void *Parameters); // 采样任务
#endif

void Task_Sys_Init(void *Parameters);     // 硬件初始化和任务初始化
void Task_Blink(void *Parameters);        // 确认存活
void Task_Chassis(void *Parameters);      // 底盘运动
void Task_Safe_Mode(void *Parameters);    // 安全模式
void Task_Gyroscope(void *Parameters);    // 陀螺仪姿态解算
void Task_Take(void *Parameters);         // 取弹
void Task_Transmission(void *Parameters); // 传动
void Task_GuideWheel(void *Parameters);
void Task_BANG(void *Parameters);
#endif
