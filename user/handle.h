#ifndef __HANDLE_H
#define __HANDLE_H

#include "OSinclude.h"
#include "Driver_Magic.h"
#include "Driver_PID.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_Motor.h"
#include "Driver_Chassis.h"
#include "Driver_Gyroscope.h"
#include "mpu6500_driver.h"
#include "mpu6500_interrupt.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// 陀螺仪
__HANDLE_EXT volatile EulerAngle_Type EulerAngle;

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type volatile Motor_LF, Motor_RF, Motor_RB, Motor_LB, Motor_SumsungLF, Motor_SumsungLB, Motor_SumsungRF, Motor_SumsungRB;
// __HANDLE_EXT volatile Motor_Type Encoder_SumsungLB, Encoder_SumsungRB, Encoder_SumsungRF, Encoder_SumsungLF;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT DBusData_Type remoteData;

// 无线串口调试
__HANDLE_EXT u8 USART_RX_BUF[MAGIC_MAX_LENGTH];
__HANDLE_EXT u16 USART_RX_STA;
__HANDLE_EXT MagicHandle_Type magic;

// Task_Chassis
__HANDLE_EXT PID_Type PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;

// Task_Sumsung
__HANDLE_EXT PID_Type ChassisAnglePID1, ChassisAnglePID2, ChassisAnglePID3, ChassisAnglePID4, CM1PID, CM2PID, CM3PID, CM4PID;

// Task_Mode_Switch
__HANDLE_EXT int sumsungMode;

void Handle_Init(void);

#endif
