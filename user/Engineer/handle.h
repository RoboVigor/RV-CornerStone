#ifndef __HANDLE_H
#define __HANDLE_H

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "rtos.h"
#include "Driver_Filter.h"
#include "Driver_Magic.h"
#include "Driver_PID.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_Motor.h"
#include "Driver_Chassis.h"
#include "Driver_Gyroscope.h"
#include "Driver_Ps.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// GPIO
#define BANG_ON GPIO_SetBits(GPIOA, GPIO_Pin_2);
#define BANG_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_2);
#define TAKE_ON GPIO_SetBits(GPIOA, GPIO_Pin_3);
#define TAKE_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_3);

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;
__HANDLE_EXT Motor_Type Motor_TakeLeft, Motor_TakeRight;
__HANDLE_EXT Motor_Type Motor_Transmission;
__HANDLE_EXT Motor_Type Motor_LGW, Motor_RGW;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT DBusData_Type remoteData;

// 陀螺仪
__HANDLE_EXT volatile EulerAngle_Type EulerAngle;

// 无线串口调试
__HANDLE_EXT u8 USART_RX_BUF[MAGIC_MAX_LENGTH];
__HANDLE_EXT u16 USART_RX_STA;
__HANDLE_EXT MagicHandle_Type magic;

// PID
__HANDLE_EXT PID_Type PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;
__HANDLE_EXT PID_Type PID_TakeLeft_Speed, PID_TakeRight_Speed, PID_TakeLeft_Angle, PID_TakeRight_Angle;
__HANDLE_EXT PID_Type PID_Transmission_Speed, PID_Transmission_Angle;
__HANDLE_EXT PID_Type PID_LGW, PID_RGW;

// 视觉数据
__HANDLE_EXT uint16_t PsData[17];

// 调试变量
__HANDLE_EXT int DebugA, DebugB, DebugC, DebugD, DebugE, DebugF, DebugG, DebugH, DebugI;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
