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
#include "mpu6500_driver.h"

#include "driver_engineer.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

/**** GPIO ****/

// Landing
#define LANDING_SWITCH_FRONT GPIO_ResetBits(GPIOH, GPIO_Pin_11);
#define LANDING_SWITCH_FRONT2 GPIO_ResetBits(GPIOI, GPIO_Pin_2);
#define LANDING_SWITCH_BEHIND GPIO_SetBits(GPIOH, GPIO_Pin_11);
#define LANDING_SWITCH_BEHIND2 GPIO_SetBits(GPIOI, GPIO_Pin_2);
#define LANDING_POWER_ON GPIO_SetBits(GPIOH, GPIO_Pin_12);
#define LANDING_POWER_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_12);

// Taking
#define ROTATE_ON GPIO_SetBits(GPIOA, GPIO_Pin_2);
#define ROTATE_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_2);

#define TAKE_ON GPIO_SetBits(GPIOA, GPIO_Pin_3);
#define TAKE_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_3);

#define TAKING_ROD_PUSH GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);
#define TAKING_ROD_PULL GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13);
#define TAKING_ROD_POWER_ON GPIO_SetBits(GPIOA, GPIO_Pin_1);
#define TAKING_ROD_POWER_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_1);

// Rescue
#define RESCUE_HOOK_DOWN GPIO_SetBits(GPIOI, GPIO_Pin_0);
#define RESCUE_HOOK_UP GPIO_ResetBits(GPIOI, GPIO_Pin_0);

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;
__HANDLE_EXT Motor_Type Motor_TakeLeft, Motor_TakeRight;
__HANDLE_EXT Motor_Type Motor_Transmission, Motor_Upthrow;
__HANDLE_EXT Motor_Type Motor_LGW, Motor_RGW;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT DBusData_Type remoteData;

// 陀螺仪
__HANDLE_EXT volatile GyrosocopeData_Type Gyroscope_EulerData;

// 无线串口调试
__HANDLE_EXT MagicHandle_Type magic;

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;

// PID
__HANDLE_EXT PID_Type PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;
__HANDLE_EXT PID_Type PID_Transmission_Speed, PID_Transmission_Angle;
__HANDLE_EXT PID_Type PID_Upthrow_Speed, PID_Upthrow_Angle;
__HANDLE_EXT PID_Type PID_LGW, PID_RGW;

// 视觉数据
__HANDLE_EXT PsData_Type PsData;

// 调试变量
__HANDLE_EXT int DebugA, DebugB, DebugC, DebugD, DebugE, DebugF, DebugG, DebugH, DebugI;
__HANDLE_EXT uint16_t DebugZ, DebugY, DebugW, DebugX;
__HANDLE_EXT int16_t Debug16_A, Debug16_B, Debug16_C, Debug16_D;

// 传感器返回值
__HANDLE_EXT u8 re_buf_Data[8], receive_ok;

// 传感器距离
__HANDLE_EXT uint16_t distance1, distance2, steady_distance1, steady_distance2;

// 取弹状态
__HANDLE_EXT int take_state, takeMode;

// 电推杆状态
__HANDLE_EXT int pushrod_state;

// 临时测试
__HANDLE_EXT uint8_t DebugJ, DebugK;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
