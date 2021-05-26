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
#include "config.h"
#include "oled.h"
#include "Driver_BSP.h"
#include "Driver_Filter.h"
#include "Driver_PID.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_Motor.h"
#include "Driver_Chassis.h"
#include "mpu6500_driver.h"
#include "Driver_Gyroscope.h"
#include "Driver_Protocol.h"
#include "Driver_Bridge.h"
#include "Driver_Magic.h"
#include "Driver_Fsm.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// Stone ID
__HANDLE_EXT uint8_t Board_Id, Robot_Id;

// 功能开关
__HANDLE_EXT uint8_t SafetyMode;

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 底盘电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;

// 机械臂
__HANDLE_EXT Motor_Type Motor_BaseJoint, Motor_ShoulderJoint, Motor_ElbowJoint, Motor_WristJoint1, Motor_WristJoint2;
__HANDLE_EXT PID_Type   PID_BaseJointSpeed, PID_BaseJointAngle,PID_ShoulderJointSpeed, PID_ShoulderJointAngle,
                        PID_ElbowSpeed, PID_ElbowAngle,PID_WristJoint1Speed, PID_WristJoint1Angle, PID_WristJoint2Speed, PID_WristJoint2Angle;
  
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

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;
__HANDLE_EXT Motor_Type       Motor_LF, Motor_RF, Motor_RB, Motor_LB;
__HANDLE_EXT PID_Type         PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;

// 通讯协议
__HANDLE_EXT ProtocolData_Type ProtocolData;
__HANDLE_EXT Node_Type         Node_Judge, Node_Host, Node_Board;

// PWM
__HANDLE_EXT PWM_Type PWM_Test;

// 总线
__HANDLE_EXT Bridge_Type BridgeData;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
