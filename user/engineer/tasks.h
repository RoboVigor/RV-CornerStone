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

// Base
void Task_Sys_Init(void *Parameters);  // 硬件初始化和任务初始化
void Task_Blink(void *Parameters);     // 确认存活
void Task_Safe_Mode(void *Parameters); // 安全模式

// Cloud
void Task_Cloud(void *Parameters);

// Sturcture
void Task_Chassis(void *Parameters);
void Task_Transmission(void *Parameters);
void Task_Pushrod(void *Parameters);

// Sensor
void Task_Distance_Sensor(void *Parameters);

// Take
void Task_Take(void *Parameters); // 取弹

// Landing
void Task_GuideWheel(void *Parameters);
void Task_BANG(void *Parameters);

// Supply
void Task_Supply(void *Parameters);

// Rescue
void Task_Rescue(void *Parameters);

#endif
