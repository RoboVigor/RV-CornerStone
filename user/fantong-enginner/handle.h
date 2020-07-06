#ifndef __HANDLE_H
#define __HANDLE_H

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "rtos.h"
#include "vegmath.h"
#include "config.h"
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
#include "Driver_Fsm.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

// Taking
#define GO_ON GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define GO_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_1)

#define GET_ON GPIO_SetBits(GPIOA, GPIO_Pin_3)
#define GET_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_3)
#define GET_STATUS GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_3)

// Rescue
#define RESCUE_HOOK_DOWN GPIO_SetBits(GPIOI, GPIO_Pin_0)
#define RESCUE_HOOK_UP GPIO_ResetBits(GPIOI, GPIO_Pin_0)

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 功能开关
__HANDLE_EXT PsAimEnabled, ChassisMode, FetchMode, RaiseMode, RescueMode;

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

// 键鼠
__HANDLE_EXT uint8_t FrictEnabled, LaserEnabled, StirEnabled, PsEnabled, AutoMode, SafetyMode;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;

// 摩擦轮
__HANDLE_EXT int Snail_State;

//平台
__HANDLE_EXT Motor_Type Motor_Fetch_X, Motor_Fetch_Left_Pitch, Motor_Fetch_Right_Pitch;

//抬升
__HANDLE_EXT Motor_Type Motor_Raise_Left, Motor_Raise_Right;

// 遥控器
__HANDLE_EXT uint8_t       remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT Remote_Type   remoteData;
__HANDLE_EXT Keyboard_Type keyboardData;
__HANDLE_EXT Mouse_Type    mouseData;

// 陀螺仪
__HANDLE_EXT volatile ImuData_Type       ImuData;
__HANDLE_EXT volatile GyroscopeData_Type Gyroscope_EulerData;

// 调试数据
__HANDLE_EXT MagicHandle_Type magic;
__HANDLE_EXT DebugData_Type   DebugData;

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;
__HANDLE_EXT PID_Type         PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;

// 云台
__HANDLE_EXT Motor_Type Motor_Yaw, Motor_Pitch;
__HANDLE_EXT PID_Type   PID_Cloud_YawAngle, PID_Cloud_YawSpeed, PID_Cloud_PitchAngle, PID_Cloud_PitchSpeed;
__HANDLE_EXT PID_Type   PID_Follow_Angle, PID_Follow_Speed;

// 抓取
__HANDLE_EXT PID_Type PID_Fetch_X;
__HANDLE_EXT PID_Type PID_Fetch_Pitch_Left, PID_Fetch_Pitch_Right;

// 上升
__HANDLE_EXT PID_Type PID_Raise_Left_Angle, PID_Raise_Left_Speed, PID_Raise_Right_Angle, PID_Raise_Right_Speed;

// 临时用
__HANDLE_EXT int16_t can1_data[12];
__HANDLE_EXT int16_t can2_data[12];

// 通讯协议
__HANDLE_EXT Protocol_Data_Type    ProtocolData;
__HANDLE_EXT Protocol_Channel_Type JudgeChannel, HostChannel, UserChannel;

// PWM
__HANDLE_EXT PWM_Type PWM_Test, PWM_Snail1, PWM_Snail2;

//发射机构
__HANDLE_EXT Motor_Type Motor_Stir, Motor_FL, Motor_FR;                     // 左/右 摩擦轮 拨弹轮 电机
__HANDLE_EXT PID_Type   PID_StirSpeed, PID_StirAngle, PID_FireL, PID_FireR; // 拨弹轮 速度/角度 PID

// 功能开关
__HANDLE_EXT uint8_t ControlMode, SafetyMode;
__HANDLE_EXT uint8_t FrictEnabled, StirEnabled, FastShootMode;

// CAN
__HANDLE_EXT Motor_Type *Can1_Device[12], *Can2_Device[12];

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
