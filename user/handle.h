#ifndef __HANDLE_H
#define __HANDLE_H

#include "OSinclude.h"
#include "Driver_Magic.h"
#include "Driver_PID.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_PanController.h"
#include "Driver_Armour.h"
#include "Driver_Angular.h"
#include "mpu6500_driver.h"
#include "mpu6500_interrupt.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// CAN
__HANDLE_EXT MotorFeedback_Type       Motor_Feedback;
__HANDLE_EXT volatile CANEncoder_Type Hook_Encoder, Armour1_Encoder, Armour2_Encoder;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT DBusData_Type remoteData;

// Magic
__HANDLE_EXT u8 USART_RX_BUF[MAGIC_MAX_LENGTH];
__HANDLE_EXT u16 USART_RX_STA;
__HANDLE_EXT MagicHandle_Type magic;

// Task_Chassis
__HANDLE_EXT PID_Type PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;

#endif
