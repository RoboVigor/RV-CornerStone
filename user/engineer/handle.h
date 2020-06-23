#ifndef __HANDLE_H
#define __HANDLE_H

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "rtos.h"
#include "vegmath.h"
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

// Constant
#define RPM2RPS ((float) 0.104667)

// Landing
#define LANDING_ON GPIO_SetBits(GPIOH, GPIO_Pin_11);
#define LANDING_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_11);

// Taking
#define TAKE_ON GPIO_SetBits(GPIOA, GPIO_Pin_3);
#define TAKE_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_3);

#define CATAPULT_ON GPIO_SetBits(GPIOA, GPIO_Pin_1);
#define CATAPULT_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_1);

// Rescue
#define RESCUE_HOOK_DOWN GPIO_SetBits(GPIOI, GPIO_Pin_0);
#define RESCUE_HOOK_UP GPIO_ResetBits(GPIOI, GPIO_Pin_0);

// 底盘运动状态
#define CHASSIS_NORMAL 0
#define CHASSIS_DETECT_RIGHT 1
#define CHASSIS_DETECT_LEFT 2
#define CHASSIS_DELANDING 3

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB;
__HANDLE_EXT Motor_Type Motor_TH, Motor_TV;
__HANDLE_EXT Motor_Type Motor_LGW, Motor_RGW;
__HANDLE_EXT Motor_Type Motor_Upthrow1, Motor_Upthrow2;
__HANDLE_EXT Motor_Type Motor_Rotate_Left, Motor_Rotate_Right;

// 遥控器
__HANDLE_EXT uint8_t remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT Remote_Type remoteData;
__HANDLE_EXT Keyboard_Type keyboardData;
__HANDLE_EXT Mouse_Type mouseData;

// 陀螺仪
__HANDLE_EXT volatile ImuData_Type       ImuData;
__HANDLE_EXT volatile GyroscopeData_Type Gyroscope_EulerData;

// 无线串口调试
__HANDLE_EXT MagicHandle_Type magic;
__HANDLE_EXT DebugData_Type DebugData;

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;

// PID
__HANDLE_EXT PID_Type PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;
__HANDLE_EXT PID_Type PID_TH_Angle, PID_TH_Speed, PID_TV_Angle, PID_TV_Speed;
__HANDLE_EXT PID_Type PID_LGW, PID_RGW;
__HANDLE_EXT PID_Type PID_Upthrow1_Angle, PID_Upthrow1_Speed, PID_Upthrow2_Angle, PID_Upthrow2_Speed;
__HANDLE_EXT PID_Type PID_Rotate_Left_Angle, PID_Rotate_Left_Speed, PID_Rotate_Right_Angle, PID_Rotate_Right_Speed;

// 通讯协议
__HANDLE_EXT Protocol_Type Judge, Ps, Board;

// 调试变量
__HANDLE_EXT int DebugA, DebugB, DebugC, DebugD, DebugE, DebugF, DebugG, DebugH, DebugI;

// 光电开关传感器数据
__HANDLE_EXT int T_State1, T_State2, T_State3, T_State4, LSR_State, LSL_State;

// 距离传感器获得距离
__HANDLE_EXT uint16_t Distance1, Distance2, Distance3, Distance4, Distance_Landing_Behind, Distance_Landing_Front, Distance_Delanding_Parallel1,
    Distance_Delanding_Parallel2, Distance_Delanding_Parallel3, Distance_Delanding_Parallel4, Distance_Delanding_Parallel5;

// 输入捕获值
__HANDLE_EXT u32 TIM5CH1_CAPTURE_VAL, TIM2CH1_CAPTURE_VAL, TIM3CH3_CAPTURE_VAL, TIM9CH1_CAPTURE_VAL;

// PWM
__HANDLE_EXT PWM_Type PWM_Supply1, PWM_Supply2, PWM_Visual_Rescue, PWM_Image_Yaw;

// Fsm需求
__HANDLE_EXT int Chassis_State, TH_Move, TU_Up, TV_Out, TR_Get, Detected_State, TH_Reset, Chassis_Detect, Chassis_Detect_Parallel, Detected_Direction, TV_Ready,
    Fsm_TIM14_Cnt, Fsm_TIM14_State, TR_Ready, Fsm_Reset, Fsm_Ready, Move_Mode, Landing_State, TH_Angle_State, TH_Ready;
__HANDLE_EXT Fsm_t Take_Fsm;
/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Take_TV_0(void);
void Take_TV_1(void);
void Take_TV_2(void);
void Handle_Init(void);
void Take_Throwup(void);
void Take_Horizontal_Right(void);
void Take_Horizontal_Left(void);
void Take_Chassis_Detect_Right(void);
void Take_Chassis_Detect_Left(void);
void Take_Start_Get(void);
void Take_TV_Progress(void);
void Take_ON(void);
void Take_Up(void);
void Take_Down(void);
void Take_Rotate_OFF(void);
void Take_OFF(void);
void Take_Catapult_On(void);
void Take_Catapult_Off(void);
void Take_Reset(void);

#endif
