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

// Taking
#define LOCK_ON GPIO_SetBits(GPIOD, GPIO_Pin_12)
#define LOCK_OFF GPIO_ResetBits(GPIOD, GPIO_Pin_12)
#define LOCK_STATE GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_12)

#define GO_OUT GPIO_SetBits(GPIOD, GPIO_Pin_13)
#define GO_BACK GPIO_ResetBits(GPIOD, GPIO_Pin_13)
#define GO_STATE GPIO_ReadOutputDataBit(GPIOD, GPIO_Pin_13)

// Rescue
#define RESCUE_HOOK_DOWN GPIO_SetBits(GPIOI, GPIO_Pin_0)
#define RESCUE_HOOK_UP GPIO_ResetBits(GPIOI, GPIO_Pin_0)

// Stone ID
__HANDLE_EXT uint8_t Board_Id, Robot_Id;

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 功能开关
__HANDLE_EXT uint8_t PsAimEnabled, ChassisMode, FetchMode, RaiseMode, RescueMode;

// 标志们
__HANDLE_EXT uint8_t Rotate1Finish, Rotate2Finish, Rotate3Finish, Rotate4Finish, AllFinish, EatFinish, GetFinish;
__HANDLE_EXT uint8_t FantongRaised, FetchState, RotateDone;
__HANDLE_EXT enum FetchStateEnum {
    FetchReset,
    FetchWaitRaise,
    FetchWaitSignal,
    FetchRotateOut,
    FetchLock,
    FetchRotateIn,
    FetchEating,
    FetchThrow,
    FetchUnlock,
    FetchDone
};

// 功能开关
__HANDLE_EXT uint8_t FrictEnabled, LaserEnabled, StirEnabled, PsEnabled, AutoMode, MilkMode, SafetyMode;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;

//抓取
__HANDLE_EXT Motor_Type Motor_Fetch_X, Motor_Fetch_Left_Pitch, Motor_Fetch_Right_Pitch;
__HANDLE_EXT PID_Type   PID_Fetch_X;
__HANDLE_EXT PID_Type   PID_Fetch_Pitch_Left, PID_Fetch_Pitch_Right;

//抬升
__HANDLE_EXT Motor_Type Motor_Raise_Left, Motor_Raise_Right;
__HANDLE_EXT PID_Type   PID_Raise_Left_Angle, PID_Raise_Left_Speed, PID_Raise_Right_Angle, PID_Raise_Right_Speed;

// 遥控器
__HANDLE_EXT uint8_t       remoteShareHost, remoteShareClient;
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

// 云台
__HANDLE_EXT Motor_Type Motor_Yaw, Motor_Pitch;
__HANDLE_EXT PID_Type   PID_Cloud_YawAngle, PID_Cloud_YawSpeed, PID_Cloud_PitchAngle, PID_Cloud_PitchSpeed;
__HANDLE_EXT PID_Type   PID_Follow_Angle, PID_Follow_Speed;

// 通讯协议
__HANDLE_EXT ProtocolData_Type ProtocolData;
__HANDLE_EXT Node_Type         Node_Judge, Node_Host, Node_Board;

// PWM
__HANDLE_EXT PWM_Type PWM_Test, PWM_Snail1, PWM_Snail2;

//发射机构
__HANDLE_EXT int        Snail_State;
__HANDLE_EXT Motor_Type Motor_Stir, Motor_FL, Motor_FR;                     // 左/右 摩擦轮 拨弹轮 电机
__HANDLE_EXT PID_Type   PID_StirSpeed, PID_StirAngle, PID_FireL, PID_FireR; // 拨弹轮 速度/角度 PID

// 补给机构
__HANDLE_EXT Motor_Type Motor_Milk;
__HANDLE_EXT PID_Type   PID_Milk_Angle, PID_Milk_Speed;

// 功能开关
__HANDLE_EXT uint8_t ControlMode, SafetyMode;
__HANDLE_EXT uint8_t FrictEnabled, StirEnabled, FastShootMode;

// 总线
__HANDLE_EXT Bridge_Type BridgeData;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
