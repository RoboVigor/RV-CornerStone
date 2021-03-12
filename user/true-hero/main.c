#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"

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
    BSP_USART6_Init(115200, USART_IT_IDLE);
    // BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_UART8_Init(115200, USART_IT_IDLE);

    // PWM
    // BSP_PWM_Set_Port(&PWM_Test, PWM_PD12);
    // BSP_PWM_Init(&PWM_Test, 9000, 200, TIM_OCPolarity_Low);

    // ADC
    BSP_ADC1_Init(1, ADC_Channel6, 0);

    // 发射机构电机
    Motor_Init(&Motor_LeftFrict, 1, DISABLE, ENABLE);
    Motor_Init(&Motor_RightFrict, 1, DISABLE, ENABLE);
    Motor_Init(&Motor_Stir3510, 19.2, DISABLE, ENABLE);

    // 云台电机
    Motor_Init(&Motor_Yaw, 1, ENABLE, ENABLE);
    Motor_Init(&Motor_Pitch, 1, ENABLE, ENABLE);

    // CAN外设
    Can1_Device[ESC_ID(0x203)] = &Motor_Yaw;
    Can1_Device[ESC_ID(0x204)] = &Motor_Stir3510;
    Can1_Device[ESC_ID(0x205)] = &Motor_LF;
    Can1_Device[ESC_ID(0x206)] = &Motor_LB;
    Can1_Device[ESC_ID(0x207)] = &Motor_RB;
    Can1_Device[ESC_ID(0x208)] = &Motor_RF;

    Can2_Device[ESC_ID(0x201)] = &Motor_LeftFrict;
    Can2_Device[ESC_ID(0x202)] = &Motor_RightFrict;
    Can2_Device[ESC_ID(0x206)] = &Motor_Pitch;

    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_Yaw);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_Stir3510);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x205, &Motor_LF);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x206, &Motor_LB);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x207, &Motor_RB);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x208, &Motor_RF);

    Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x201, &Motor_LeftFrict);
    Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x202, &Motor_RightFrict);
    Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x206, &Motor_Pitch);

    // 陀螺仪设置静态误差
    Gyroscope_Set_Bias(&ImuData, 0.60, 34.65, -9.85);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // // 初始化串口调试数据
    // Magic_Init(&magic, 0);

    // 通讯协议初始化
    Protocol_Init(&Node_Judge, &ProtocolData);
    Protocol_Init(&Node_Host, &ProtocolData);
    Protocol_Init(&Node_Board, &ProtocolData);

    // 总线设置
    // Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_LF);
    // Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_LB);
    // Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_RB);
    // Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_RF);
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x501, &Node_Host);
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x502, &Node_Board);
    // Bridge_Bind(&BridgeData, USART_BRIDGE, 6, &Node_Judge);
    // Bridge_Bind(&BridgeData, USART_BRIDGE, 7, &Node_Board);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 8, &Node_Host);

    // 陀螺仪
    Gyroscope_Set_Bias(&ImuData, -4, 3, 1);    // 设置静态误差
    Gyroscope_Init(&Gyroscope_EulerData, 300); // 初始化

    /*******************************************************************************
     *                                 任务初始化                                   *
     *******************************************************************************/

    // 低级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    xTaskCreate(Task_Fire, "Task_Fire", 400, NULL, 5, NULL);

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 4, NULL);

    // 通讯
    // xTaskCreate(Task_Client_Communication, "Task_Client_Communication", 500, NULL, 6, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 6, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

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
    // Bridge_Send_Protocol(&Node_Host, 0x120, 1); // 心跳包

    //启动调度,开始执行任务
    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while (1) {
    }
}
