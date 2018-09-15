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

// Dbus
__HANDLE_EXT uint8_t DBusBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT DBusData_Type DBusData, LastDBusData;

// PanController
__HANDLE_EXT PID_Type CM1PID, CM2PID, CM3PID, CM4PID, YawAnglePID, YawSpeedPID1, YawSpeedPID2, YawSpeedPID;
__HANDLE_EXT ChassisParam_Type ChassisParam;
__HANDLE_EXT float             targetYawAngle, yawAngleFeed, yawSpeedFeed;

// Magic
__HANDLE_EXT u8 USART_RX_BUF[MAGIC_MAX_LENGTH];
__HANDLE_EXT u16 USART_RX_STA;
__HANDLE_EXT MagicHandle_Type magic;

// RTOS
__HANDLE_EXT TaskHandle_t TaskHandle_DBus;
__HANDLE_EXT TaskHandle_t TaskHandle_Debug;
__HANDLE_EXT TaskHandle_t TaskHandle_Blink;
__HANDLE_EXT TaskHandle_t TaskHandle_Chassis;
__HANDLE_EXT TaskHandle_t TaskHandle_Safe_Mode;

__HANDLE_EXT QueueHandle_t Queue_Test;

#endif
