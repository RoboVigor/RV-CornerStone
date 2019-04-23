#ifndef __TASK_SYSTEMINIT_H
#define __TASK_SYSTEMINIT_H

#include "BSP_CAN.h"
#include "BSP_DMA.h"
#include "BSP_GPIO.h"
#include "BSP_NVIC.h"
#include "BSP_TIM.h"
#include "BSP_UART.h"

#if DEBUG_ENABLED
void Task_Debug_Magic_Send(void *Parameters);         // 无线串口调试输出
void Task_Debug_Magic_Receive(void *Parameters);      // 无线串口调试接收
void Task_Debug_RTOS_State(void *Parameters);         // 输出RTOS系统资源
void Task_Debug_Gyroscope_Sampling(void *Parameters); // 采样任务
#endif

/**** Base ****/

void Task_Sys_Init(void *Parameters);  // 硬件初始化和任务初始化
void Task_Blink(void *Parameters);     // 确认存活
void Task_Safe_Mode(void *Parameters); // 安全模式

/**** Function ****/

// Cloud
void Task_Cloud(void *Parameters);
void Task_Monitor(void *Parameters);
// Chassis
void Task_Chassis(void *Parameters);
// Landing
void Task_Landing_GuideWheel(void *Parameters); // 引导轮
void Task_Landing_BANG(void *Parameters);       // 登岛气动控制
// Taking
void Task_Take(void *Parameters);                // 取弹装置
void Task_Taking_Transmission(void *Parameters); // 传动装置
void Task_Distance_Sensor(void *Parameters);     // 距离传感器
void Task_Get_Distance(void *Parameters);         // 距离传感器
void Task_Taking_Pushrod(void *Parameters);      // 电推杆
// Supply
void Task_Supply(void *Parameters);
// Rescue
void Task_Rescue(void *Parameters);

#endif
