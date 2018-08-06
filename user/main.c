#include "main.h"
#include "Task_SysInitConfig.h"

int main(void)
{
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

	//创建系统初始化任务
    xTaskCreate(Task_SysInitConfig,
                "SysInitConfig",
                1600,
                NULL,
                5,
                NULL);
	
	  //任务开始
    vTaskStartScheduler();

	while (1);
}
