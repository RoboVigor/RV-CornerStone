#ifndef __DRIVER_Armour_H
#define __DRIVER_Armour_H

#define ARM2MINUSARM1 149.458

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
	

}ArmourPID_Type;




#ifdef  __Armour_Global
#define __Armour_EXT  
#else
#define __Armour_EXT extern
#endif


__Armour_EXT ArmourPID_Type Armour1_SpeedPID,Armour2_SpeedPID,Armour1_AnglePID,Armour2_AnglePID,Armour1_SinglePID_SpeedPID,Armour2_SinglePID_SpeedPID;
__Armour_EXT float Armour1TargetAngle,Armour2TargetAngle,Armour1FeedAngle,Armour2FeedAngle,SinglePID_TargetSpeed;

void ArmourAnglePIDInit(ArmourPID_Type * pid, float Kp,float Ki, float Kd);

void ArmourSpeedPIDInit(ArmourPID_Type * pid, float Kp,float Ki, float Kd);
	
void ArmourSpeedPID(ArmourPID_Type * pid, int speed, int feed);

void ArmourAnglePID(ArmourPID_Type * pid, int angle, int feed);

void ArmourTargetAngleInit(void);




#endif
