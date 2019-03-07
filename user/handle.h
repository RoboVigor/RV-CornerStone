#ifndef __HANDLE_H
#define __HANDLE_H

#include "OSinclude.h"
#include "Driver_Magic.h"
#include "Driver_PID.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_Motor.h"
#include "Driver_Chassis.h"
#include "Driver_Armour.h"
#include "Driver_Angular.h"
#include "Driver_Referee.h"
#include "mpu6500_driver.h"
#include "mpu6500_interrupt.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT DBusData_Type remoteData;

// Referee System
__HANDLE_EXT uint8_t JudgeDataBuffer[JudgeBufferLength];
__HANDLE_EXT extGameRobotState_t Judge_RobotState;
__HANDLE_EXT extShootData_t Judge_ShootData;
__HANDLE_EXT extPowerHeatData_t Judge_PowerHeatData;

// 无线串口调试
__HANDLE_EXT u8 USART_RX_BUF[MAGIC_MAX_LENGTH];
__HANDLE_EXT u16 USART_RX_STA;
__HANDLE_EXT MagicHandle_Type magic;

// Task_Chassis
__HANDLE_EXT PID_Type PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;

// Task_Fire
__HANDLE_EXT Motor_Type Motor_LeftFrict, Motor_RightFrict, Motor_Stir; // 左/右 摩擦轮 拨弹轮 电机
__HANDLE_EXT PID_Type PID_LeftFrictSpeed, PID_RightFrictSpeed;         // 左/右 摩擦轮 PID
__HANDLE_EXT PID_Type PID_StirSpeed, PID_StirAnlge;                    // 拨弹轮 速度/角度 PID
__HANDLE_EXT uint8_t isStirSwitchOn;

void Handle_Init(void);

#endif
