/**
 * @brief 哨兵
 * @version 0.8.0
 */
#include "main.h"

void Task_Safe_Mode(void *Parameters) {
  while (1) {
    if (remoteData.switchRight == 2) {
      Can_Send(CAN1, 0x200, 0, 0, 0, 0);
      vTaskSuspendAll();
    }
    vTaskDelay(100);
  }
  vTaskDelete(NULL);
}

// int debug1 = 0;

void Task_Chassis(void *Parameters) {
  TickType_t LastWakeTime = xTaskGetTickCount(); // 时钟
  float rpm2rps =
      0.104667f; // 转子的转速(round/min)换算成角速度(rad/s) = 2 * 3.14 / 60

  // 初始化角速度PID
  PID_Init(&PID_Motor_Chassis_Left, 15, 0, 0, 4000, 2000);
  PID_Init(&PID_Motor_Chassis_Right, 15, 0, 0, 4000, 2000);
  PID_Init(&PID_Stabilizer_Yaw_Speed, 5, 0, 0, 4000, 2000);

  while (1) {
    // 计算输出电流PID
    PID_Calculate(&PID_LFCM, (float)remoteData.lx,
                  Motor_Chassis_Left.speed * rpm2rps);
    PID_Calculate(&PID_RFCM, (float)remoteData.lx,
                  Motor_Chassis_Right.speed * rpm2rps);
    PID_Calculate(&PID_Stabilizer_Yaw_Speed, (float)remoteData.rx,
                  Motor_Stabilizer_Yaw.speed * rpm2rps);

    // 输出电流值到电调
    Can_Send(CAN1, 0x200, PID_Motor_Chassis_Left.output,
             PID_Motor_Chassis_Right.output, PID_Stabilizer_Yaw_Speed.output,
             0);

    // 底盘运动更新频率
    vTaskDelayUntil(&LastWakeTime, 10);
  }

  vTaskDelete(NULL);
}

void Task_Debug_Magic_Send(void *Parameters) {
  TickType_t LastWakeTime = xTaskGetTickCount();

  while (1) {
    taskENTER_CRITICAL(); // 进入临界段
    printf("Yaw: %f \r\n", Gyroscope_EulerData.yaw);
    taskEXIT_CRITICAL(); // 退出临界段
    vTaskDelayUntil(&LastWakeTime, 500);
  }
  vTaskDelete(NULL);
}

void Task_Sys_Init(void *Parameters) {

  // 初始化全局变量
  Handle_Init();

  // BSP们
  BSP_GPIO_Init();
  BSP_CAN_Init();
  BSP_UART_Init();
  BSP_DMA_Init();
  BSP_TIM_Init();
  BSP_NVIC_Init();
  BSP_USER_Init();

  // 初始化陀螺仪
  MPU6500_Initialize();
  MPU6500_EnableInt();

  // 遥控器数据初始化
  DBUS_Init(&remoteData);

  //陀螺仪计数器开启确认

  // 调试任务
#if DEBUG_ENABLED
  Magic_Init_Handle(&magic, 0); // 初始化调试数据的默认值
  xTaskCreate(Task_Debug_Magic_Receive, "Task_Debug_Magic_Receive", 500, NULL,
              6, NULL);
  // xTaskCreate(Task_Debug_Magic_Send, "Task_Debug_Magic_Send", 500, NULL, 6,
  //                                                 NULL);
  // xTaskCreate(Task_Debug_RTOS_State, "Task_Debug_RTOS_State", 500, NULL, 6,
  // NULL); xTaskCreate(Task_Debug_Gyroscope_Sampling,
  // "Task_Debug_Gyroscope_Sampling", 400, NULL, 6, NULL);
#endif

  // 功能任务
  xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
  xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
  xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 3, NULL);

  // 完成使命
  vTaskDelete(NULL);
}

void Task_Blink(void *Parameters) {
  TickType_t LastWakeTime = xTaskGetTickCount();
  while (1) {
    GREEN_LIGHT_TOGGLE;
    vTaskDelayUntil(&LastWakeTime, 250);
  }

  vTaskDelete(NULL);
}
