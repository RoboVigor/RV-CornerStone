#ifndef __HANDLE_H
#define __HANDLE_H

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "rtos.h"
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

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// Constant
#define RPM2RPS ((float) 0.104667)

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
__HANDLE_EXT Motor_Type Motor_Transmission;
__HANDLE_EXT Motor_Type Motor_LGW, Motor_RGW;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT DBusData_Type remoteData;

// 陀螺仪
__HANDLE_EXT volatile ImuData_Type       ImuData;
__HANDLE_EXT volatile GyroscopeData_Type Gyroscope_EulerData;

// 无线串口调试
__HANDLE_EXT MagicHandle_Type magic;

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;

// PID
__HANDLE_EXT PID_Type PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;
__HANDLE_EXT PID_Type PID_TakeLeft_Speed, PID_TakeRight_Speed, PID_TakeLeft_Angle, PID_TakeRight_Angle;
__HANDLE_EXT PID_Type PID_Transmission_Speed, PID_Transmission_Angle;
__HANDLE_EXT PID_Type PID_LGW, PID_RGW;

// 通讯协议
__HANDLE_EXT Protocol_Type Judge, Ps;

// 调试变量
__HANDLE_EXT int DebugA, DebugB, DebugC, DebugD, DebugE, DebugF, DebugG, DebugH, DebugI;

// 取弹状态
__HANDLE_EXT int takeMode, State;

// 距离传感器获得距离
__HANDLE_EXT uint16_t Distance1, Distance2;

// 输入捕获值
__HANDLE_EXT u32 TIM5CH1_CAPTURE_VAL, TIM2CH1_CAPTURE_VAL;

// PWM
__HANDLE_EXT PWM_Type PWM_Supply1, PWM_Supply2, PWM_Image_Yaw, PWM_Image_Pitch, PWM_Rescue;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
