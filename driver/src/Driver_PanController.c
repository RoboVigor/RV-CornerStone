#define __PanController_Global
#define ABS(x) ((x)>=0?(x):-(x))


#include "Driver_PanController.h"
#include "mpu6500_interrupt.h"
#include "Driver_Angular.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"

#include "led.h"

int LastMode = 0;

void Chassis_Init_Yaw_Angle(void)
{
	targetYawAngle = 0;
}

/**
 * @brief  地盘yaw角度
 * @param
	*
 * @return PID输出Wv 转动速度
 */

void PanYawSpeedPID(PANPID_Type * pid, int angle, int feed)
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
 * @brief  地盘yaw角度
 * @param
	*
 * @return PID输出Wv 转动速度
 */

void PanAnglePID(PANPID_Type * pid, int angle, int feed)
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
 * @brief 	底盘电机3508速度PID控制
 * @param  void
 * @return void
 */

void PID_Set_Pan_Speed(PANPID_Type * pid, int speed, int feed)
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
void PANSpeedPIDInit(PANPID_Type * pid, float Kp,float Ki, float Kd)
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

	pid->IMax = 4000;
	pid->PIDMax = 8000;//2750
	pid->Iindex = 1;
	pid->PIDindex = 1;

	pid->TargetAngle = 0;
	pid->TargetSpeed = 0;
	pid->AngleFeed = 0;
	pid->SpeedFeed = 0;
}

void PanYawSpeedPIDInit(PANPID_Type * pid, float Kp,float Ki, float Kd)
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

	pid->IMax = 330;
	pid->PIDMax = 660;//2750
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
void PANAnglePIDInit(PANPID_Type * pid, float Kp,float Ki, float Kd)
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

	pid->IMax = 330;
	pid->PIDMax = 660;//2750
	pid->Iindex = 1;
	pid->PIDindex = 1;

	pid->TargetAngle = 0;
	pid->TargetSpeed = 0;
	pid->AngleFeed = 0;
	pid->SpeedFeed = 0;
}
void Chassis_Set_Wheel_Speed(int XSpeed, int YSpeed,int WSpeed)
{
    ChassisParam.TargetVX = (float)XSpeed/660*5;
    ChassisParam.TargetVY = (float)YSpeed/660*5;
	  ChassisParam.TargetWR = (float)WSpeed/660*15;//4*1.5*2
}
void Chassis_Update_Mecanum_Data(int buffer[4])
{
			float K = 0.946;
			buffer[0] = 13.16*((ChassisParam.TargetVX)-(ChassisParam.TargetVY)+ChassisParam.TargetWR*(-K))*19.2;//麦克解算遥控器来的数值来礵e
			buffer[1] = 13.16*((ChassisParam.TargetVX)+(ChassisParam.TargetVY)+ChassisParam.TargetWR*(-K))*19.2;//转子的角速度/19=轮子角速度 (rad/s)
			buffer[2] = -13.16*(ChassisParam.TargetVX-(ChassisParam.TargetVY)+ChassisParam.TargetWR*K)*19.2;
			buffer[3] = -13.16*((ChassisParam.TargetVX)+(ChassisParam.TargetVY)+ChassisParam.TargetWR*K)*19.2;

}

void Chassis_Get_XYW_Speed(int dir, int Mode)
{

			yawSpeedFeed = mpu6500_data.gz/16.4;

			//if(ABS(DbusData.ch1)<5)
			if(Mode == 2)
			{
				if(LastMode != 2)
				{
					YawAnglePID.Iout = 0;
					YawSpeedPID1.Iout = 0;
					YawSpeedPID2.Iout = 0;
					targetYawAngle = yawAngleFeed;
				}



				PanAnglePID(&YawAnglePID,targetYawAngle,yawAngleFeed);//??pid?? ??????
				PanYawSpeedPID(&YawSpeedPID1,YawAnglePID.PIDout,yawSpeedFeed);

				Chassis_Set_Wheel_Speed(-DbusData.ch4*dir,DbusData.ch3*dir,YawSpeedPID1.PIDout);
				LastMode = 2;
			}
			else if(Mode == 1)
			{
				if(LastMode != 1)
				{
					YawAnglePID.Iout = 0;
					YawSpeedPID1.Iout = 0;
					YawSpeedPID2.Iout = 0;
				}

				PanYawSpeedPID(&YawSpeedPID2,-DbusData.ch1/6,yawSpeedFeed);

				Chassis_Set_Wheel_Speed(-DbusData.ch4*dir,DbusData.ch3*dir,YawSpeedPID2.PIDout);


				LastMode = 1;
			}
			else if(Mode == 0)
			{
				Can_Set_Motor_Speed(CAN1, 0, 0, 0, 0);


			}


}

void Chassis_Limit_Wheel_Speed(int WheelSpeedOrigin[4],int WheelSpeedRes[4],int MaxWheelSpeed)
{
	float MaxSpeed = 0;
	float Param = 0;
	int index = 0;

	for(; index < 4; index++)
	{
			if(ABS(WheelSpeedOrigin[index]) > MaxSpeed)
			{
					MaxSpeed = ABS(WheelSpeedOrigin[index]);
			}
	}

		if(MaxWheelSpeed < MaxSpeed)
		{
			Param = (float)MaxWheelSpeed / MaxSpeed;
			WheelSpeedRes[0] = WheelSpeedOrigin[0] * Param;
			WheelSpeedRes[1] = WheelSpeedOrigin[1] * Param;
			WheelSpeedRes[2] = WheelSpeedOrigin[2] * Param;
			WheelSpeedRes[3] = WheelSpeedOrigin[3] * Param;
		}
		else
		{
			WheelSpeedRes[0] = WheelSpeedOrigin[0];
			WheelSpeedRes[1] = WheelSpeedOrigin[1];
			WheelSpeedRes[2] = WheelSpeedOrigin[2];
			WheelSpeedRes[3] = WheelSpeedOrigin[3];
		}

}

