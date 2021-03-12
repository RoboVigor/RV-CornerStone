#ifndef __HANDLE_H
#define __HANDLE_H

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "vegmath.h"
#include "stdlib.h"
#include "limits.h"
#include "Driver_BSP.h"
#include "Driver_Filter.h"
#include "Driver_PID.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_Motor.h"
#include "Driver_Chassis.h"
#include "mpu6500_driver.h"
#include "Driver_Gyroscope.h"
#include "Driver_Bridge.h"
#include "Driver_Magic.h"
#include "Driver_Protocol.h"
#include "Driver_Fsm.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

//
__HANDLE_EXT uint8_t Board_Id, Robot_Id;

// 键鼠
__HANDLE_EXT uint8_t FrictEnabled, LaserEnabled, StirEnabled, PsEnabled, AutoMode, SafetyMode;

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_Chassis_Left, Motor_Chassis_Right;

// 云台
__HANDLE_EXT Motor_Type Motor_Up_Gimbal_Yaw, Motor_Up_Gimbal_Pitch, Motor_Down_Gimbal_Yaw, Motor_Down_Gimbal_Pitch;
__HANDLE_EXT PID_Type   PID_Up_Gimbal_Yaw_Angle, PID_Up_Gimbal_Yaw_Speed, PID_Up_Gimbal_Pitch_Angle, PID_Up_Gimbal_Pitch_Speed;
__HANDLE_EXT PID_Type   PID_Down_Gimbal_Yaw_Angle, PID_Down_Gimbal_Yaw_Speed, PID_Down_Gimbal_Pitch_Angle, PID_Down_Gimbal_Pitch_Speed;

// 摩擦轮
__HANDLE_EXT Motor_Type Motor_Up_Frict_Left, Motor_Up_Frict_Right, Motor_Down_Frict_Left, Motor_Down_Frict_Right;
__HANDLE_EXT PID_Type   PID_Up_Frict_Left_Speed, PID_Up_Frict_Right_Speed, PID_Down_Frict_Left_Speed, PID_Down_Frict_Right_Speed;

// 遥控器
__HANDLE_EXT uint8_t       remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT Remote_Type   remoteData;
__HANDLE_EXT Keyboard_Type keyboardData;
__HANDLE_EXT Mouse_Type    mouseData;

// 陀螺仪
__HANDLE_EXT volatile ImuData_Type       ImuData;
__HANDLE_EXT volatile GyroscopeData_Type Gyroscope_EulerData;

// 调试数据
__HANDLE_EXT DebugData_Type DebugData;

__HANDLE_EXT int debug1, debug2, debug3;

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;
__HANDLE_EXT PID_Type         PID_Chassis_Left, PID_Chassis_Right;

// 通讯协议
__HANDLE_EXT ProtocolData_Type ProtocolData;
__HANDLE_EXT Node_Type         Node_Judge, Node_Host, Node_Board;

// 发射机构
__HANDLE_EXT Motor_Type Motor_Up_Stir, Motor_Down_Stir;                                                 // 左/右 摩擦轮 拨弹轮 电机
__HANDLE_EXT PID_Type   PID_Up_Stir_Speed, PID_Up_Stir_Angle, PID_Down_Stir_Speed, PID_Down_Stir_Angle; // 拨弹轮 速度/角度 PID

// 光电开关传感器数据
__HANDLE_EXT int Left_State, Right_State;

// CAN
__HANDLE_EXT Bridge_Type BridgeData;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
