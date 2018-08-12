#define __HookController_Global

#include "Driver_HookController.h"

void HookTargetAngleInit(void)
{
	HookTargetAngle = 0;
}


/**
 * @brief 	钩子的减速电机3508角度PID控制
 * @param  void
 * @return void
 */
void HookAnglePID(HookPID_Type * pid, int angle, int feed)
{	
	pid->TargetAngle=angle;
	pid->AngleFeed=feed;
	pid->CurrentError = pid->TargetAngle - pid->AngleFeed;	
	
	pid->Pout = pid->P * pid->CurrentError;

	pid->Iout += pid->I * pid->CurrentError;
	pid->Iout = pid->Iout > pid->IMax ? pid->IMax : pid->Iout;
	pid->Iout = pid->Iout < -pid->IMax ? -pid->IMax : pid->Iout;
	
	pid->PIDout = (pid->Pout + pid->Iout + pid->Dout);
	
	pid->PIDout = pid->PIDout > pid->PIDMax ? pid->PIDMax : pid->PIDout;
	pid->PIDout = pid->PIDout < -pid->PIDMax ? -pid->PIDMax : pid->PIDout;
	
	pid->LastError = pid->CurrentError;
}




/**
 * @brief 	钩子的减速电机3508速度PID控制
 * @param  void
 * @return void
 */

void HookSpeedPID(HookPID_Type * pid, int speed, int feed)
{	
	pid->TargetSpeed=speed;
	pid->SpeedFeed=feed;
	pid->CurrentError = pid->TargetSpeed - pid->SpeedFeed;	
	
	pid->Pout = pid->P * pid->CurrentError;

	pid->Iout += pid->I * pid->CurrentError;
	pid->Iout = pid->Iout > pid->IMax ? pid->IMax : pid->Iout;
	pid->Iout = pid->Iout < -pid->IMax ? -pid->IMax : pid->Iout;
	
	pid->PIDout = (pid->Pout + pid->Iout + pid->Dout);
	
	pid->PIDout = pid->PIDout > pid->PIDMax ? pid->PIDMax : pid->PIDout;
	pid->PIDout = pid->PIDout < -pid->PIDMax ? -pid->PIDMax : pid->PIDout;
	
	pid->LastError = pid->CurrentError;
}
/**
 * @brief  PID参数初始化
 * @param  PID_Type结构体、PID三个参数
 * @return void
 */
void HookSpeedPIDInit(HookPID_Type * pid, float Kp,float Ki, float Kd)
{
	pid->P = Kp;
	pid->I = Ki;
	pid->D = Kd;
	
	pid->CurrentError = 0;
	pid->LastError = 0;
	pid->Pout = 0;
	pid->Iout = 0;
	pid->Dout = 0;
	pid->PIDout = 0;
	
	pid->IMax = 1250;
	pid->PIDMax = 2500;//2750
	pid->Iindex = 1;
	pid->PIDindex = 1;
	
	pid->TargetAngle = 0;
	pid->TargetSpeed = 0;
	pid->AngleFeed = 0;
	pid->SpeedFeed = 0;
}
/**
 * @brief  PID参数初始化
 * @param  PID_Type结构体、PID三个参数
 * @return void
 */
void HookAnglePIDInit(HookPID_Type * pid, float Kp,float Ki, float Kd)
{
	pid->P = Kp;
	pid->I = Ki;
	pid->D = Kd;
	
	pid->CurrentError = 0;
	pid->LastError = 0;
	pid->Pout = 0;
	pid->Iout = 0;
	pid->Dout = 0;
	pid->PIDout = 0;
	
	pid->IMax = 1250;
	pid->PIDMax = 2500;//2750
	pid->Iindex = 1;
	pid->PIDindex = 1;
	
	pid->TargetAngle = 0;
	pid->TargetSpeed = 0;
	pid->AngleFeed = 0;
	pid->SpeedFeed = 0;
}


