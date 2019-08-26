#ifndef __HANDLE_H
#define __HANDLE_H

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "rtos.h"
#include "vegmath.h"
#include "stdlib.h"
#include "limits.h"
#include "Driver_BSP.h"
#include "Driver_Filter.h"
#include "Driver_Magic.h"
#include "Driver_PID.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_Motor.h"
#include "Driver_Chassis.h"
#include "mpu6500_driver.h"
#include "Driver_Gyroscope.h"
#include "Driver_Protocol.h"
#include "Driver_Fsm.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// 键鼠
__HANDLE_EXT uint8_t FrictEnabled, LaserEnabled, StirEnabled, PsEnabled, AutoMode, SafetyMode;

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_Chassis_Left, Motor_Chassis_Right;

// 云台
__HANDLE_EXT Motor_Type Motor_Stabilizer_Yaw, Motor_Stabilizer_Pitch;
__HANDLE_EXT PID_Type PID_Stabilizer_Yaw_Angle, PID_Stabilizer_Yaw_Speed;
__HANDLE_EXT PID_Type PID_Stabilizer_Pitch_Angle, PID_Stabilizer_Pitch_Speed;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT Remote_Type remoteData;
__HANDLE_EXT Keyboard_Type keyboardData;
__HANDLE_EXT Mouse_Type mouseData;

// 陀螺仪
__HANDLE_EXT volatile ImuData_Type       ImuData;
__HANDLE_EXT volatile GyroscopeData_Type Gyroscope_EulerData;

// 调试数据
__HANDLE_EXT MagicHandle_Type magic;
__HANDLE_EXT DebugData_Type DebugData;
__HANDLE_EXT int            debug1, debug2, debug3;

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;
__HANDLE_EXT PID_Type PID_Chassis_Left, PID_Chassis_Right;

// 通讯协议
__HANDLE_EXT Protocol_Type Judge, Ps;

// 发射机构
__HANDLE_EXT Motor_Type Motor_Stir;                   // 左/右 摩擦轮 拨弹轮 电机
__HANDLE_EXT PID_Type PID_Stir_Speed, PID_Stir_Angle; // 拨弹轮 速度/角度 PID

// PWM
__HANDLE_EXT PWM_Type PWM_Snail1, PWM_Snail2;

// 光电开关传感器数据
__HANDLE_EXT int Left_State, Right_State;

// 摩擦轮
__HANDLE_EXT int Snail_State;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
