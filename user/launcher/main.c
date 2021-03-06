#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"

void BSP_Switch_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
}

int main(void) {

    //设置中断优先级位数
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //调试相关
    Delay_Init(180); // 初始化延时
    LED_Init();      // 初始化LED
    Beep_Init();     // 初始化蜂鸣器

    /*******************************************************************************
     *                                  硬件初始化                                  *
     *******************************************************************************/

    // 获得设备ID
    BSP_Stone_Id_Init(&Board_Id, &Robot_Id);

    // 卷绳电机
    Motor_Init(&Motor_Charge, 1, DISABLE, ENABLE);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 通讯协议初始化
    Protocol_Init(&Node_Judge, &ProtocolData);
    Protocol_Init(&Node_Host, &ProtocolData);
    Protocol_Init(&Node_Board, &ProtocolData);

    // 硬件配置
    BSP_CAN_Init();
    BSP_DBUS_Init(remoteBuffer);
    BSP_TIM2_Init();
    BSP_IMU_Init();
    BSP_Laser_Init();
    BSP_Beep_Init();
    BSP_LED_Init();
    BSP_User_Power_Init();
    BSP_Switch_Init();

    // USART
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_UART7_Init(115200, USART_IT_IDLE);
    // BSP_UART8_Init(115200, USART_IT_IDLE);

    // PWM
    BSP_PWM_Set_Port(&PWM_Hook_L, PWM_PA1);
    BSP_PWM_Init(&PWM_Hook_L, 9000, 200, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Hook_R, PWM_PA2);
    BSP_PWM_Init(&PWM_Hook_R, 9000, 200, TIM_OCPolarity_Low);

    // ADC

    // 总线设置
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_Charge);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 6, &Node_Judge);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 7, &Node_Board);
    // Bridge_Bind(&BridgeData, USART_BRIDGE, 8, &Node_Host);

    /*******************************************************************************
     *                                 任务初始化                                   *
     *******************************************************************************/

    // 低优先级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    // while (!remoteData.state) {
    // }

    // 高优先级任务
    xTaskCreate(Task_Charge, "Task_Charge", 400, NULL, 6, NULL);               // 发射架蓄能任务
    xTaskCreate(Task_Hook, "Task_Hook", 400, NULL, 6, NULL);                   // 载飞镖模块固定与释放任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);             // 遥控器控制任务
    xTaskCreate(Task_Communication, "Task_Communication", 500, NULL, 6, NULL); // 通讯任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 6, NULL);           // Can发送任务

    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while (1) {
    }
}
