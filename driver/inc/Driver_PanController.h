#ifndef __DRIVER_PanController_H
#define __DRIVER_PanController_H

#include "stm32f4xx.h"



typedef struct 
{
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
	

}PANPID_Type;

typedef struct
{
	float TargetVX;
	float TargetVY;
	float TargetWR;
	int WheelSpeed[4];
}ChassisParamTypeDef;


#ifdef  __PanController_Global
#define __PanController_EXT  
#else
#define __PanController_EXT extern
#endif


__PanController_EXT PANPID_Type CM1PID,CM2PID,CM3PID,CM4PID,YawAnglePID,YawSpeedPID1,YawSpeedPID2;
__PanController_EXT ChassisParamTypeDef ChassisParam;
__PanController_EXT float TargetYawAngle,YawAngleFeed,YawSpeedFeed;

void PANAnglePIDInit(PANPID_Type * pid, float Kp,float Ki, float Kd);

void GetXYWSpeed(int dir, int Mode);

void LimitWheelSpeed(int WheelSpeedOrigin[4],int WheelSpeedRes[4],int MaxWheelSpeed);

void PANSpeedPIDInit(PANPID_Type * pid, float Kp,float Ki, float Kd);
	
void PANSpeedPID(PANPID_Type * pid, int speed, int feed);

void Chassis_SpeedSet(int XSpeed, int YSpeed,int WSpeed);

void PanAnglePID(PANPID_Type * pid, int angle, int feed);

void PanYawSpeedPID(PANPID_Type * pid, int angle, int feed);

void PanTargetAngleInit(void);

void MecanumCalculation(int buffer[4]);
void PanYawSpeedPIDInit(PANPID_Type * pid, float Kp,float Ki, float Kd);



#endif
