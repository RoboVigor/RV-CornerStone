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

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_Yaw, Motor_Pitch, Motor_Stir, Motor_Roll;
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

// pid
__HANDLE_EXT ChassisData_Type ChassisData;
__HANDLE_EXT PID_Type PID_Cloud_YawAngle, PID_Cloud_YawSpeed, PID_Cloud_PitchAngle, PID_Cloud_PitchSpeed, PID_StirSpeed, PID_StirAngle, PID_Cloud_RollAngle,
    PID_Cloud_RollSpeed;

// 通讯协议
__HANDLE_EXT ProtocolData_Type ProtocolData;
__HANDLE_EXT Node_Type         Node_Judge, Node_Host, Node_Board;

// PWM
__HANDLE_EXT PWM_Type PWM_Test, PWM_Snail1, PWM_Snail2, PWM_Servo;

// PWM 捕获
__HANDLE_EXT u32 TIM5CH1_CAPTURE_STA, TIM5CH1_CAPTURE_VAL;

// 键鼠模式切换
__HANDLE_EXT int controlMode;

// 功能开关
__HANDLE_EXT uint8_t FrictEnabled, StirEnabled, PsAimEnabled, PsShootEnabled, SafetyMode;

__HANDLE_EXT int snailStart; // snail 开启标志位 1为开 0为关

__HANDLE_EXT int debug1, debug2;

// 总线
__HANDLE_EXT Bridge_Type BridgeData;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
