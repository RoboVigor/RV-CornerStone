#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"
// #include "queue.h"

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

    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, DISABLE);

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
    // BSP_OLED_Init();

    // USART
    // BSP_USART3_Init(115200, USART_IT_IDLE); // Rx故障
    // BSP_USART6_Init(115200, USART_IT_IDLE); // Tx故障
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_UART8_Init(115200, USART_IT_IDLE);

    // PWM
    // BSP_PWM_Set_Port(&PWM_Test, PWM_PD12);
    // BSP_PWM_Init(&PWM_Test, 9000, 200, TIM_OCPolarity_Low);

    // ADC
    BSP_ADC1_Init(1, ADC_Channel6, 0);

    // 总线设置
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_LF);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_LB);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_RB);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_RF);
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x501, &Node_Host);
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x502, &Node_Board);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 7, &Node_Host);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 8, &Node_Judge);

    // 陀螺仪
    Gyroscope_Set_Bias(&ImuData, 0, 0, 0);     // 设置静态误差
    Gyroscope_Init(&Gyroscope_EulerData, 300); // 初始化

    /*******************************************************************************
     *                                 任务初始化                                   *
     *******************************************************************************/

    // 低优先级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL); // 跑马灯/呼吸灯任务
    // xTaskCreate(Task_OLED, "Task_OLED", 400, NULL, 3, NULL);// OLED 菜单任务
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);// 开机音乐任务

    // 等待遥控器开启
    // while (!remoteData.state) {
    // }

    // 高优先级任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL); //模式切换任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL); // 底盘运动任务
    // xTaskCreate(Task_Communication, "Task_Communication", 500, NULL, 6, NULL); // 通讯测试任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 6, NULL); // Can发送任务

    // 定义协议发送频率
    Bridge_Send_Protocol(&Node_Host, 0x120, 1); // 心跳包
    // Bridge_Send_Protocol(&Node_Host, 0x501, 50); // 板间通讯

    //启动调度,开始执行任务
    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while (1) {
    }
}
