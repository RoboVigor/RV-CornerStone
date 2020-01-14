#ifndef __HANDLE_H
#define __HANDLE_H

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "rtos.h"
#include "vegmath.h"
#include "config.h"
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

// Taking
#define GO_ON GPIO_SetBits(GPIOA, GPIO_Pin_3);
#define GO_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_3);

#define GET_ON GPIO_SetBits(GPIOA, GPIO_Pin_1);
#define GET_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_1);

// Rescue
#define RESCUE_HOOK_DOWN GPIO_SetBits(GPIOI, GPIO_Pin_0);
#define RESCUE_HOOK_UP GPIO_ResetBits(GPIOI, GPIO_Pin_0);

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 功能开关
__HANDLE_EXT PsAimEnabled;
__HANDLE_EXT uint8_t ControlMode;
__HANDLE_EXT uint8_t GoMode, GetMode, UpMode, RotateMode;

// 标志们
__HANDLE_EXT uint8_t Rotate1Finish, Rotate2Finish, Rotate3Finish, Rotate4Finish, AllFinish, EatFinish, GetFinish;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;
__HANDLE_EXT Motor_Type Motor_Fetch_X, Motor_Fetch_LP, Motor_Fetch_RP;
__HANDLE_EXT Motor_Type Motor_Upthrow1, Motor_Upthrow2;

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

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;
__HANDLE_EXT PID_Type PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;

// 抓取
__HANDLE_EXT PID_Type PID_Fetch_X;
__HANDLE_EXT PID_Type PID_Fetch_Pitch_Left, PID_Fetch_Pitch_Right;

// 上升
__HANDLE_EXT PID_Type PID_Upthrow1_Angle, PID_Upthrow1_Speed, PID_Upthrow2_Angle, PID_Upthrow2_Speed;

// 通讯协议
__HANDLE_EXT Protocol_Type Judge, Ps;

// PWM
__HANDLE_EXT PWM_Type PWM_Test;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
