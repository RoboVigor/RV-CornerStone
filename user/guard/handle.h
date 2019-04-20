#ifndef __HANDLE_H
#define __HANDLE_H

#include "Driver_CAN.h"
#include "Driver_Chassis.h"
#include "Driver_DBUS.h"
#include "Driver_Filter.h"
#include "Driver_Gyroscope.h"
#include "Driver_Magic.h"
#include "Driver_Motor.h"
#include "Driver_PID.h"
#include "Driver_Ps.h"
#include "beep.h"
#include "delay.h"
#include "key.h"
#include "led.h"
#include "mpu6500_driver.h"
#include "rtos.h"
#include "sys.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_Chassis_Left, Motor_Chassis_Right, Motor_Stabilizer_Yaw, Motor_Stabilizer_Pitch;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT DBusData_Type remoteData;

// 陀螺仪
__HANDLE_EXT volatile GyrosocopeData_Type Gyroscope_EulerData;

// 无线串口调试
__HANDLE_EXT MagicHandle_Type magic;

// 底盘
__HANDLE_EXT PID_Type PID_Chassis_Left, PID_Chassis_Right;

// 云台
__HANDLE_EXT PID_Type PID_Stabilizer_Yaw_Angle, PID_Stabilizer_Yaw_Speed;
__HANDLE_EXT PID_Type PID_Stabilizer_Pitch_Angle, PID_Stabilizer_Pitch_Speed;

// 视觉数据
__HANDLE_EXT PsData_Type PsData;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif