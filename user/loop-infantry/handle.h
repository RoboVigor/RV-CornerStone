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

#define CHARGE_ON GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define CHARGE_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_4)

#define DISCHARGE_ON GPIO_SetBits(GPIOA, GPIO_Pin_5)
#define DISCHARGE_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_5)

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 功能开关
__HANDLE_EXT uint8_t ControlMode;
__HANDLE_EXT uint8_t FrictEnabled, StirEnabled, SwingMode, PsAimEnabled, PsShootEnabled, LowSpeedMode, MagzineOpened, FastShootMode, SafetyMode, PigeonMode;
//鸽子模式是开超级电容的模式（鸽子是世界上跑的最快的生物！逃）万一武工不同意，备选名称：WuwuwuMode，XiuxiuMode

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
__HANDLE_EXT MagicHandle_Type magic;
__HANDLE_EXT DebugData_Type   DebugData;

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;
__HANDLE_EXT PID_Type         PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed;
__HANDLE_EXT uint8_t          PigeonCurrent, PigeonVoltage, PigeonEnergy, PigeonChargeEnable;
//超级电容的电压和裁判系统输出电流，为AD转换后的电压值。后面的是剩余电量,以及通过底盘功率对充电时间进行限制

// 通讯协议
__HANDLE_EXT Protocol_Type Judge, Ps;

// 弹舱盖舵机
__HANDLE_EXT PWM_Type PWM_Magazine_Servo;

//发射机构
__HANDLE_EXT Motor_Type Motor_Stir;                   // 左/右 摩擦轮 拨弹轮 电机
__HANDLE_EXT PID_Type   PID_StirSpeed, PID_StirAngle; // 拨弹轮 速度/角度 PID

// PWM
__HANDLE_EXT PWM_Type PWM_Test, PWM_Snail1, PWM_Snail2;

// ADC
__HANDLE_EXT uint32_t ADC_Value[ADC_CHANNEL_NUM][ADC_CHANNEL_NUM]; // 20次，两个通道,1为电压，2为电流
/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

#endif
