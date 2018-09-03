#ifndef __Driver_HookController_H
#define __Driver_HookController_H

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

} HookPID_Type;

#ifdef __HookController_Global
#define __HookController_EXT
#else
#define __HookController_EXT extern
#endif

__HookController_EXT HookPID_Type Hook_SpeedPID, Hook_AnglePID;
__HookController_EXT float        HookRotateSpeed, HookTargetAngle, HookFeedAngle;

void HookSpeedPIDInit(HookPID_Type *pid, float Kp, float Ki, float Kd);

void HookAnglePIDInit(HookPID_Type *pid, float Kp, float Ki, float Kd);

void HookSpeedPID(HookPID_Type *pid, int speed, int feed);

void Chassis_Set_Wheel_Speed(int XSpeed, int YSpeed, int WSpeed);

void HookAnglePID(HookPID_Type *pid, int angle, int feed);

void HookTargetAngleInit(void);

#endif
