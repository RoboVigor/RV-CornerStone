#define __HANDLER_GLOBALS

#include "main.h"

void Task_SysInit(void *Parameters) {
  BSP_GPIO_InitConfig();
  BSP_CAN_InitConfig();
  BSP_UART_InitConfig();
  BSP_DMA_InitConfig();
  BSP_UART_InitConfig();
  BSP_TIM_InitConfig();
  BSP_NVIC_InitConfig();

  delay_init(180); // 延时初始化
  uart_init(9600); // 初始化串口
  LED_Init();      // 初始化LED
  BEEP_Init();     // 初始化蜂鸣器

  TIM4_PWM_Init();
  MPU6500_IntConfiguration();

  delay_ms(3000);
  Hook_Encoder.ecd_bias = Motor_Feedback.Motor_205_Agree;
  GetEncoderBias(&Hook_Encoder);
  Armour1_Encoder.ecd_bias = Motor_Feedback.Motor_206_Agree;
  Armour2_Encoder.ecd_bias = Motor_Feedback.Motor_207_Agree;
  GetEncoderBias(&Armour1_Encoder);
  GetEncoderBias(&Armour2_Encoder);

  MPU6500_Initialize();

  delay_ms(3000);
  MPU6500_EnableInt();

  //  mpu6500 offset
  mpu6500_data.gx_offset = (short)0.406409323;
  mpu6500_data.gy_offset = (short)-2.91589163;
  mpu6500_data.gz_offset = (short)15.75639464;

  // 目标值初始化
  HookTargetAngleInit();
  PanTargetAngleInit();
  ArmourTargetAngleInit();

  PanYawSpeedPIDInit(&YawSpeedPID1, 2, 0, 0);
  PanYawSpeedPIDInit(&YawSpeedPID2, 2, 0, 0);
  // 底盘角度pid初始化
  PANAnglePIDInit(&YawAnglePID, 15, 0, 0);

  // 底盘速度pid初始化
  PANSpeedPIDInit(&CM1PID, 12, 0, 0);
  PANSpeedPIDInit(&CM2PID, 12, 0, 0);
  PANSpeedPIDInit(&CM3PID, 12, 0, 0);
  PANSpeedPIDInit(&CM4PID, 12, 0, 0);

  // 钩子角度速度pid初始化
  HookAnglePIDInit(&Hook_AnglePID, 0, 0, 0);
  HookSpeedPIDInit(&Hook_SpeedPID, 2.1, 0.2, 0);

  // 装甲板两个电机角度速度pid初始化
  ArmourAnglePIDInit(&Armour1_AnglePID, 0, 0, 0);
  ArmourAnglePIDInit(&Armour2_AnglePID, 0, 0, 0);
  ArmourSpeedPIDInit(&Armour1_SpeedPID, 0, 0, 0);
  ArmourSpeedPIDInit(&Armour2_SpeedPID, 0, 0, 0);

  // 装甲板单速度pid
  ArmourSpeedPIDInit(&Armour1_SinglePID_SpeedPID, 1.1, 0, 0);
  ArmourSpeedPIDInit(&Armour2_SinglePID_SpeedPID, 1, 0, 0);

  // 建立任务
  //// IRQ任务
  xTaskCreate(Task_USART3, "Task_USART3", 400, NULL, 6, &TaskHandler_USART3);
  xTaskCreate(Task_DBUS, "Task_DBUS", 400, NULL, 6, &TaskHandler_DBUS);
  //// 低优先级任务
  xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, &TaskHandler_Blink);

  vTaskDelete(NULL);
}
