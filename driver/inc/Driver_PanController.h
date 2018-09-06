#ifndef __DRIVER_PanController_H
#define __DRIVER_PanController_H

#include "stm32f4xx.h"

typedef struct {
    float P;
    float I;
    float D;

    int CurrentError;
    int LastError;
    int Pout;
    int Iout;
    int Dout;
    int PIDout;

    int IMax;
    int PIDMax;
    int Iindex;
    int PIDindex;

    int LastPIDOut;

    int AngleFeed;
    int SpeedFeed;
    int TargetAngle;
    int TargetSpeed;
    int Portion;

} PANPID_Type;

typedef struct {
    float TargetVX;
    float TargetVY;
    float TargetWR;
    int   WheelSpeed[4];
} ChassisParam_Type;

#ifdef __PanController_Global
#define __PanController_EXT
#else
#define __PanController_EXT extern
#endif

__PanController_EXT PANPID_Type CM1PID, CM2PID, CM3PID, CM4PID, YawAnglePID, YawSpeedPID1, YawSpeedPID2,YawSpeedPID;
__PanController_EXT ChassisParam_Type ChassisParam;
__PanController_EXT float               targetYawAngle, yawAngleFeed, yawSpeedFeed;

void PANAnglePIDInit(PANPID_Type *pid, float Kp, float Ki, float Kd);
void PanYawSpeedPIDInit(PANPID_Type *pid, float Kp, float Ki, float Kd);
void PANSpeedPIDInit(PANPID_Type *pid, float Kp, float Ki, float Kd);
void PanAnglePID(PANPID_Type *pid, int angle, int feed);
void PanYawSpeedPID(PANPID_Type *pid, int angle, int feed);

void Chassis_Get_XYW_Speed(int dir, int Mode);

void Chassis_Limit_Wheel_Speed(int WheelSpeedOrigin[4], int WheelSpeedRes[4], int MaxWheelSpeed);

void PID_Set_Pan_Speed(PANPID_Type *pid, int speed, int feed);

void Chassis_Set_Wheel_Speed(int XSpeed, int YSpeed, int WSpeed);

void Chassis_Init_Yaw_Angle(void);

void Chassis_Update_Mecanum_Data(int buffer[4]);

#endif
