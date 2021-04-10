#ifndef __HANDLE_H
#define __HANDLE_H

#include "Driver_BSP.h"
#include "Driver_CAN.h"
#include "Driver_Chassis.h"
#include "Driver_DBUS.h"
#include "Driver_Filter.h"
#include "Driver_Fsm.h"
#include "Driver_Gyroscope.h"
#include "Driver_Motor.h"
#include "Driver_PID.h"
#include "Driver_Protocol.h"
#include "Driver_Bridge.h"
#include "Driver_Magic.h"
#include "beep.h"
#include "delay.h"
#include "key.h"
#include "led.h"
#include "mpu6500_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "sys.h"
#include "vegmath.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// Stone ID
__HANDLE_EXT uint8_t Board_Id, Robot_Id;

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;

// 云台
__HANDLE_EXT Motor_Type Motor_Yaw, Motor_Pitch;
__HANDLE_EXT PID_Type   PID_Cloud_YawAngle, PID_Cloud_YawSpeed, PID_Cloud_PitchAngle, PID_Cloud_PitchSpeed;
__HANDLE_EXT PID_Type   PID_Follow_Angle, PID_Follow_Speed;

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
__HANDLE_EXT PID_Type         PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;

// 通讯协议
__HANDLE_EXT ProtocolData_Type ProtocolData;
__HANDLE_EXT Node_Type         Node_Judge, Node_Host, Node_Board;

//发射机构
__HANDLE_EXT Motor_Type Motor_LeftFrict, Motor_RightFrict, Motor_Stir3510; // 左/右 摩擦轮 拨弹轮电机
__HANDLE_EXT PID_Type   PID_LeftFrictSpeed, PID_RightFrictSpeed, PID_Stir3510Speed, PID_Stir3510Angle, PID_Stir2006Angle,
    PID_Compensation; // 拨弹轮 速度/角度 PID

// 功能开关
__HANDLE_EXT uint8_t ControlMode;
__HANDLE_EXT uint8_t ShootEnabled, PsEnabled, UpEnabled, ServoEnabled, StirStop, SafetyMode, SwingMode, ShootMode;

// 总线
__HANDLE_EXT Bridge_Type BridgeData;

// CAN
__HANDLE_EXT Motor_Type *Can1_Device[12], *Can2_Device[12];

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
