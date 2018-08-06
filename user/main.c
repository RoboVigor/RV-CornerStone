#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "beep.h"
#include "key.h"
#include "BSP_GPIO.h"
#include "BSP_NVIC.h"
#include "BSP_UART.h"
#include "BSP_DMA.h"
#include "BSP_CAN.h"
#include "BSP_TIM.h"
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "Driver_PanController.h"
#include "Driver_HookController.h"
#include "Driver_Armour.h"
#include "Driver_Angular.h"
#include "mpu6500_interrupt.h"
#include "Task_SysInitConfig.h"

#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define FORWARD 1
#define BACKWARD -1
#define MAXWHEELSPEED 1150

int32_t debug_YawAngleFeed = 0;
int32_t debug_value1 = 0;
int32_t debug_value2 = 0;
int32_t debug_value3 = 0;
int32_t debug_value4 = 0;
int32_t debug_value5 = 0;
int32_t debug_value6 = 0;
int32_t debug_value7 = 0;
int32_t debug_value8 = 0;

int debug_tim = 0;

void Task_SysInitConfig(void *Parameters);
int main(void)
{
	//创建系统初始化任务
    xTaskCreate(Task_SysInitConfig,
                "SysInitConfig",
                1600,
                NULL,
                5,
                NULL);
	
	  //任务开始
    vTaskStartScheduler();
	
	int ArmatureRotateSpeed[4], Buffer[4];
	int isDoublePID = 0; //0为单速度pid

	int LastMode = 0; //1指上次为直线前进模式 2上次为自转模式

	float LastYawAngleFeed = 0;
	float YawAngleFeedOffset = 0;
	float YawAngleFeedThreshold = 0.003;
	float YawAngleFeedDiff = 0;
	float YawAngleFeedOffsetSample = 0;
	float YawAngleFeedOffsetSampleCounter = 0;
	int PanPIDMode = 1;

	BSP_GPIO_InitConfig();
	BSP_CAN_InitConfig();
	BSP_UART_InitConfig();
	BSP_DMA_InitConfig();
	BSP_UART_InitConfig();
	BSP_TIM_InitConfig();
	BSP_NVIC_InitConfig();

	delay_init(180);   //延时初始化
	uart_init(115200); //串口初始化波特率为115200
	LED_Init();		   //初始化与LED连接的硬件接口
	BEEP_Init();

	TIM4_PWM_Init();
	MPU6500_IntConfiguration();

	//	printf("============%d=============",Hook_Encoder.ecd_bias);
	delay_ms(3000);
	Hook_Encoder.ecd_bias = Motor_Feedback.Motor_205_Agree;
	GetEncoderBias(&Hook_Encoder);
	Armour1_Encoder.ecd_bias = Motor_Feedback.Motor_206_Agree;
	Armour2_Encoder.ecd_bias = Motor_Feedback.Motor_207_Agree;
	GetEncoderBias(&Armour1_Encoder); //得到编码器的初始bias
	GetEncoderBias(&Armour2_Encoder);

	RED_LIGHT_OFF;
	GREEN_LIGHT_OFF;

	MPU6500_Initialize();

	delay_ms(3000);
	MPU6500_EnableInt();

	mpu6500_data.gx_offset = (short)0.406409323;
	mpu6500_data.gy_offset = (short)-2.91589163;
	mpu6500_data.gz_offset = (short)15.75639464;

	HookTargetAngleInit();
	PanTargetAngleInit();
	ArmourTargetAngleInit();

	PanYawSpeedPIDInit(&YawSpeedPID1, 2, 0, 0); //3
	PanYawSpeedPIDInit(&YawSpeedPID2, 2, 0, 0); //3
	PANAnglePIDInit(&YawAnglePID, 15, 0, 0);	//底盘角度pid初始化25

	PANSpeedPIDInit(&CM1PID, 12, 0, 0); //底盘速度pid初始化 12  0.01    22 0.17
	PANSpeedPIDInit(&CM2PID, 12, 0, 0);
	PANSpeedPIDInit(&CM3PID, 12, 0, 0);
	PANSpeedPIDInit(&CM4PID, 12, 0, 0);

	HookAnglePIDInit(&Hook_AnglePID, 0, 0, 0); //钩子角度速度pid初始化
	HookSpeedPIDInit(&Hook_SpeedPID, 2.1, 0.2, 0);

	ArmourAnglePIDInit(&Armour1_AnglePID, 0, 0, 0); //装甲板两个电机角度速度pid初始化
	ArmourAnglePIDInit(&Armour2_AnglePID, 0, 0, 0);
	ArmourSpeedPIDInit(&Armour1_SpeedPID, 0, 0, 0);
	ArmourSpeedPIDInit(&Armour2_SpeedPID, 0, 0, 0);

	ArmourSpeedPIDInit(&Armour1_SinglePID_SpeedPID, 1.1, 0, 0); //装甲板单速度pid
	ArmourSpeedPIDInit(&Armour2_SinglePID_SpeedPID, 1, 0, 0);

	while (1)
	{

		if (debug_tim % 2000 > 1000)
		{
			RED_LIGHT_ON;
			GREEN_LIGHT_ON;
		}
		else
		{
			RED_LIGHT_OFF;
			GREEN_LIGHT_OFF;
		}

		//printf("123");
		if (DBUS_ReceiveData.switch_right == 2)
		{
			Set_CM_Speed(CAN1, 0, 0, 0, 0); //得到电流发送给电调
			continue;
		}
		//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

		Motion_Update();
		/*
		if(DBUS_ReceiveData.switch_right == 1)
		{
				YawSpeedPID2.P = 2.3;
		}
		else if(DBUS_ReceiveData.switch_right == 3)
		{
				YawSpeedPID2.P = 1.7;
		}*/

		if (ABS(DBUS_ReceiveData.ch1) < 5)
		{
			PanPIDMode = 2;
		}
		else
		{
			PanPIDMode = 1;
		}

		if (Euler_Angle.Yaw - YawAngleFeedOffset - LastYawAngleFeed > 300)
		{
			YawAngleFeedOffset += 360;
		}
		else if (Euler_Angle.Yaw - YawAngleFeedOffset - LastYawAngleFeed < -300)
		{
			YawAngleFeedOffset -= 360;
		}

		YawAngleFeed = Euler_Angle.Yaw - YawAngleFeedOffset;

		YawAngleFeedDiff = YawAngleFeed - LastYawAngleFeed;

		if (ABS(DBUS_ReceiveData.ch1) < 10 && ABS(DBUS_ReceiveData.ch3) < 10 && ABS(DBUS_ReceiveData.ch4) < 10)
		//if(1)
		{
			if (ABS(YawAngleFeedDiff) < YawAngleFeedThreshold)
			{
				YawAngleFeedOffset += YawAngleFeedDiff;
				YawAngleFeed = LastYawAngleFeed;
				PanPIDMode = 0;
				if (YawAngleFeedOffsetSampleCounter < 100)
				{
					YawAngleFeedOffsetSample += YawAngleFeedDiff;
					YawAngleFeedOffsetSampleCounter += 1;
				}
			}
			else
			{
				LastYawAngleFeed = YawAngleFeed;
			}
		}
		else
		{
			if (DBUS_ReceiveData.switch_right == 1)
			{
				YawAngleFeedOffset += YawAngleFeedOffsetSample / YawAngleFeedOffsetSampleCounter;
			}
			YawAngleFeed = Euler_Angle.Yaw - YawAngleFeedOffset;
			YawAngleFeedDiff = YawAngleFeed - LastYawAngleFeed;
			LastYawAngleFeed = YawAngleFeed;
		}
		/*DEBUGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG*/
		debug_YawAngleFeed = (int)(YawAngleFeed);
		debug_value1 = (int)(DBUS_ReceiveData.ch1);
		debug_value2 = (int)(DBUS_ReceiveData.ch2);
		debug_value3 = (int)(DBUS_ReceiveData.ch3);
		debug_value4 = (int)(DBUS_ReceiveData.ch4);
		debug_value5 = (int)(YawAngleFeedOffsetSample);
		debug_value6 = (int)(YawAngleFeedOffsetSampleCounter);
		debug_value7 = (int)(YawAngleFeedOffset * 10000);
		debug_value8 = (int)(YawAngleFeedOffsetSample / YawAngleFeedOffsetSampleCounter * 10000);

		if (DBUS_ReceiveData.switch_left == 1) //摄像头朝向丝杆 功能：移动
		{

			TIM_SetCompare1(TIM4, 23);
			//			-------------------------------------------------------------------------------------------------------------------------------

			EncoderProcess(&Hook_Encoder, Motor_Feedback.Motor_205_Agree);

			HookSpeedPID(&Hook_SpeedPID, 0, Motor_Feedback.Motor_205_Speed);

			Set_Hook_Armour_Speed(CAN2, Hook_SpeedPID.PIDout, 0, 0, 0);

			//			-------------------------------------------------------------------------------------------------------------------------------

			GetXYWSpeed(FORWARD, PanPIDMode);

			MecanumCalculation(Buffer);

			LimitWheelSpeed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

			//速度pid
			PANSpeedPID(&CM1PID, ArmatureRotateSpeed[0], Motor_Feedback.Motor_201_Speed * 2 * 3.14 / 60);
			PANSpeedPID(&CM2PID, ArmatureRotateSpeed[1], Motor_Feedback.Motor_202_Speed * 2 * 3.14 / 60);
			PANSpeedPID(&CM3PID, ArmatureRotateSpeed[2], Motor_Feedback.Motor_203_Speed * 2 * 3.14 / 60);
			PANSpeedPID(&CM4PID, ArmatureRotateSpeed[3], Motor_Feedback.Motor_204_Speed * 2 * 3.14 / 60); //都是rad/s 反馈转子转速

			Set_CM_Speed(CAN1, CM1PID.PIDout, CM2PID.PIDout, CM3PID.PIDout, CM4PID.PIDout); //得到电流发送给电调
		}
		//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		else if (DBUS_ReceiveData.switch_left == 3)
		{
			TIM_SetCompare1(TIM4, 5);

			EncoderProcess(&Hook_Encoder, Motor_Feedback.Motor_205_Agree);

			HookFeedAngle = Hook_Encoder.ecd_angle;

			HookSpeedPID(&Hook_SpeedPID, DBUS_ReceiveData.ch2, Motor_Feedback.Motor_205_Speed);

			Set_Hook_Armour_Speed(CAN2, Hook_SpeedPID.PIDout, 0, 0, 0);

			//=======移动=============================================================================

			GetXYWSpeed(BACKWARD, PanPIDMode);

			MecanumCalculation(Buffer);

			LimitWheelSpeed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

			//速度pid
			PANSpeedPID(&CM1PID, ArmatureRotateSpeed[0], Motor_Feedback.Motor_201_Speed * 2 * 3.14 / 60);
			PANSpeedPID(&CM2PID, ArmatureRotateSpeed[1], Motor_Feedback.Motor_202_Speed * 2 * 3.14 / 60);
			PANSpeedPID(&CM3PID, ArmatureRotateSpeed[2], Motor_Feedback.Motor_203_Speed * 2 * 3.14 / 60);
			PANSpeedPID(&CM4PID, ArmatureRotateSpeed[3], Motor_Feedback.Motor_204_Speed * 2 * 3.14 / 60); //都是rad/s 反馈转子转速

			Set_CM_Speed(CAN1, CM1PID.PIDout, CM2PID.PIDout, CM3PID.PIDout, CM4PID.PIDout); //得到电流发送给电调
		}
		//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		else if (DBUS_ReceiveData.switch_left == 2)
		{
			//			if(isDoublePID == 1)
			//			{

			//				EncoderProcess(&Armour1_Encoder,Motor_Feedback.Motor_206_Agree);
			//				EncoderProcess(&Armour2_Encoder,Motor_Feedback.Motor_207_Agree);
			//
			//				Armour1FeedAngle =	Armour1_Encoder.ecd_angle;
			//				Armour2FeedAngle = 	Armour2_Encoder.ecd_angle;
			//
			//
			//				printf(" 202: %f    \r\n",Armour1_Encoder.ecd_angle);
			//				printf(" 203: %f    \r\n",Armour2_Encoder.ecd_angle);
			//
			//				ArmourAnglePID(&Armour1_AnglePID,Armour1TargetAngle,Armour1FeedAngle/19);
			//				ArmourAnglePID(&Armour2_AnglePID,Armour2TargetAngle,Armour2FeedAngle/19);
			//
			//				ArmourSpeedPID(&Armour1_SpeedPID,Armour1_AnglePID.PIDout,Motor_Feedback.Motor_206_Speed*2*3.14/60);
			//				ArmourSpeedPID(&Armour2_SpeedPID,Armour2_AnglePID.PIDout,Motor_Feedback.Motor_207_Speed*2*3.14/60);
			//
			//
			//				Set_Hook_Armour_Speed(CAN2,0,DBUS_ReceiveData.ch2*2,DBUS_ReceiveData.ch4*3,0);
			//			}
			//			else if(isDoublePID == 0)
			//			{

			//				EncoderProcess(&Armour1_Encoder,Motor_Feedback.Motor_206_Agree);
			//				EncoderProcess(&Armour2_Encoder,Motor_Feedback.Motor_207_Agree);
			//
			//				printf(" 202: %d    \r\n",Armour1_Encoder.round_cnt);
			//				printf(" 203: %d    \r\n",Armour2_Encoder.round_cnt);
			//				if(1)
			//				{
			//					//printf("---206::--------%f------------\r\n",Motor_Feedback.Motor_206_Speed*2*3.14/60);
			//					//printf("---target::--------%f------------\r\n",SinglePID_TargetSpeed);
			//					SinglePID_TargetSpeed = DBUS_ReceiveData.ch2*2;
			//					ArmourSpeedPID(&Armour1_SinglePID_SpeedPID,SinglePID_TargetSpeed,Motor_Feedback.Motor_206_Speed*2*3.14/60);
			//					ArmourSpeedPID(&Armour2_SinglePID_SpeedPID,SinglePID_TargetSpeed*2,Motor_Feedback.Motor_207_Speed*2*3.14/60);
			//
			//					if(DBUS_ReceiveData.switch_right == 3)//正常
			//						Set_Hook_Armour_Speed(CAN2,0,Armour1_SinglePID_SpeedPID.PIDout,Armour2_SinglePID_SpeedPID.PIDout,0);
			//					else if(DBUS_ReceiveData.switch_right == 2)//遥控器分别控制调试
			//						Set_Hook_Armour_Speed(CAN2,0, DBUS_ReceiveData.ch2*4, DBUS_ReceiveData.ch4*4,0);
			//
			//				}
			//				else
			//				{
			//					Set_Hook_Armour_Speed(CAN2,0,0,0,0);
			//				}
			//			}
		}

		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

		delay_ms(1);
	}
}
