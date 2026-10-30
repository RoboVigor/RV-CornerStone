#ifndef __HANDLE_H
#define __HANDLE_H

#include "sys.h"
#include "delay.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
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
#include "Driver_Vofa.h"
#include "unitree.h"
#include "Driver_MecArm.h"

#ifdef __HANDLE_GLOBALS
#define __HANDLE_EXT
#else
#define __HANDLE_EXT extern
#endif

//rtos任务句柄
__HANDLE_EXT TaskHandle_t MecArmTask_Handler;

// Stone Id
__HANDLE_EXT uint8_t Board_Id, Robot_Id;

// TIM
__HANDLE_EXT volatile uint32_t ulHighFrequencyTimerTicks;

// 功能开关
__HANDLE_EXT uint8_t ControlMode;
__HANDLE_EXT uint8_t FrictEnabled, StirEnabled, MagzineOpened;
__HANDLE_EXT uint8_t PsAimEnabled, PsShootEnabled;
__HANDLE_EXT uint8_t SwingMode, SafetyMode, PigeonMode;
 typedef enum {normalMove = 0, fastMove = 1, frictMove = 2}moveMode_t;
 typedef enum {normalShoot = 0, fastShoot = 1, frictShoot = 2}ShootMode_t;
__HANDLE_EXT moveMode_t moveMode;
__HANDLE_EXT ShootMode_t ShootMode;

// 上位机
__HANDLE_EXT uint8_t        FacingEnemyMode;
__HANDLE_EXT autoaim_data_t HostAutoaimData;
__HANDLE_EXT chassis_data_t HostChassisData;

// 电机
__HANDLE_EXT Motor_Type Motor_LF, Motor_RF, Motor_RB, Motor_LB, Motor_LAJI;

// 云台
__HANDLE_EXT Motor_Type Motor_Yaw, Motor_Pitch;
__HANDLE_EXT PID_Type   PID_Cloud_YawAngle, PID_Cloud_YawSpeed, PID_Cloud_PitchAngle, PID_Cloud_PitchSpeed, PID_Cloud_MotorYawSpeed;
__HANDLE_EXT PID_Type   PID_Follow_Angle, PID_Follow_Speed;

// 遥控器
__HANDLE_EXT uint8_t       remoteBuffer[DBUS_LENGTH + DBUS_BACK_LENGTH];
__HANDLE_EXT Remote_Type   remoteData;
__HANDLE_EXT Keyboard_Type keyboardData;
__HANDLE_EXT Mouse_Type    mouseData;

// 陀螺仪
__HANDLE_EXT volatile ImuData_Type       ImuData;
__HANDLE_EXT volatile GyroscopeData_Type Gyroscope_EulerData;
__HANDLE_EXT SemaphoreHandle_t ImuDataReady;

// 调试数据
__HANDLE_EXT DebugData_Type *DebugData;
__HANDLE_EXT VofaData_type *VofaData;

// 底盘
__HANDLE_EXT ChassisData_Type ChassisData;
__HANDLE_EXT PID_Type         PID_LFCM, PID_LBCM, PID_RBCM, PID_RFCM, PID_YawAngle, PID_YawSpeed, PID_Fx, PID_Fy, PID_T, PID_Power;
__HANDLE_EXT uint8_t          PigeonCurrent, PigeonVoltage, PigeonEnergy, PigeonChargeEnable;

// 通讯协议
__HANDLE_EXT ProtocolData_Type ProtocolData;
__HANDLE_EXT Node_Type         Node_Judge, Node_Host, Node_Board, Node_SuperCap, Node_Debug;

// 弹舱盖舵机
__HANDLE_EXT PWM_Type PWM_Magazine_Servo;

//发射机构
__HANDLE_EXT Motor_Type Motor_Stir, Motor_FL, Motor_FR;                     // 左/右 摩擦轮 拨弹轮 电机
__HANDLE_EXT PID_Type   PID_StirSpeed, PID_StirAngle, PID_FireL, PID_FireR; // 拨弹轮 速度/角度 PID

// PWM
__HANDLE_EXT PWM_Type PWM_Test;

// CAN
__HANDLE_EXT Bridge_Type BridgeData;

//Vofa
__HANDLE_EXT Vofa_Debug_JustFloat_Type vf_debug;

//机械臂
__HANDLE_EXT Motor_Type Motor6020_MecArm_0_0;
__HANDLE_EXT Motor_Type Motor6020_MecArm_0_1;
__HANDLE_EXT PID_Type Motor6020_MecArm_Group_0_PosPID;
__HANDLE_EXT PID_Type Motor6020_MecArm_Group_0_SpeedPID;
__HANDLE_EXT Motor_Group_Type Motor6020_MecArm_Group_0;

__HANDLE_EXT Unitree_Motor_Type Unitree_MecArm_1;
__HANDLE_EXT Unitree_Motor_Type Unitree_MecArm_2;
__HANDLE_EXT Unitree_Bridge_Type Unitree_Bridge;

__HANDLE_EXT Motor_Type Motor6020_MecArm_3;
__HANDLE_EXT PID_Type Motor6020_MecArm_3_PosPID;
__HANDLE_EXT PID_Type Motor6020_MecArm_3_SpeedPID;

__HANDLE_EXT Arm_Joint_Type Arm_Joint_0;
__HANDLE_EXT Arm_Joint_Type Arm_Joint_1;
__HANDLE_EXT Arm_Joint_Type Arm_Joint_2;
__HANDLE_EXT Arm_Joint_Type Arm_Joint_3;
__HANDLE_EXT Arm_Joint_Type Arm_Joint_4;

__HANDLE_EXT Manipulator_Type MecArm;

/**
 * @brief 初始化结构体
 * @note 该函数将在所有硬件及任务初始化之前执行
 */
void Handle_Init(void);

// 初始化事件组，用于需要各类需要先初始化的业务且刚需放置再rtos中进行，保证初始化完成才正常进入到完整控制系统中
__HANDLE_EXT EventGroupHandle_t InitEventGroup;

#endif
