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
    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);

    // 发射机构电机
    Motor_Init(&Motor_Stir, STIR_MOTOR_REDUCTION_RATE, ENABLE, ENABLE); //拨弹
    Motor_Init(&Motor_FL, 1, DISABLE, ENABLE);
    Motor_Init(&Motor_FR, 1, DISABLE, ENABLE);

    // 云台电机
    Motor_Init(&Motor_Yaw, GIMBAL_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);   // 顺时针为正电流
    Motor_Init(&Motor_Pitch, GIMBAL_MOTOR_REDUCTION_RATE, ENABLE, ENABLE); // 顺时针为正电流

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

    // 获得设备ID
    BSP_Stone_Id_Init(&Board_Id, &Robot_Id);

    // USART
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_UART8_Init(115200, USART_IT_IDLE);

    // Servo
    BSP_PWM_Set_Port(&PWM_Magazine_Servo, PWM_PI0);
    BSP_PWM_Init(&PWM_Magazine_Servo, 9000, 200, TIM_OCPolarity_Low);

    if (ROBOT_MIAO) {
        Motor_Yaw.positionBias   = 4110;
        Motor_Yaw.position       = 4110;
        Motor_Pitch.positionBias = 5540;
        Motor_Pitch.position     = 5540;
        Gyroscope_Set_Bias(&ImuData, 13, -22, -18);
    } else if (ROBOT_WANG) {
        Motor_Yaw.positionBias   = 5427;
        Motor_Yaw.position       = 5427;
        Motor_Pitch.positionBias = 2628;
        Motor_Pitch.position     = 2628;
        Gyroscope_Set_Bias(&ImuData, -8, -16, -7);
    }

    // 总线设置

    if (ROBOT_MIAO) {
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_LF);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_LB);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_RB);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_RF);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x206, &Motor_Pitch);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x209, &Motor_Yaw);
        Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x201, &Motor_FL);
        Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x202, &Motor_FR);
        Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x207, &Motor_Stir);
    } else if (ROBOT_WANG) {
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_LF);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_LB);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_RB);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_RF);
        Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x209, &Motor_Pitch);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x209, &Motor_Yaw);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x206, &Motor_FL);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x205, &Motor_FR);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x207, &Motor_Stir);
    }

    // 总线设置
    Bridge_Bind(&BridgeData, USART_BRIDGE, 7, &Node_Host);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 8, &Node_Judge);

    // 陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData, 300); // 初始化

    /*******************************************************************************
     *                                 任务初始化                                   *
     *******************************************************************************/

    // 等待遥控器开启
    while (!remoteData.state) {
    }
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 5, NULL);

    // 运动控制任务
    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
    xTaskCreate(Task_Fire_Stir, "Task_Fire_Stir", 400, NULL, 6, NULL);
    xTaskCreate(Task_Fire_Frict, "Task_Fire_Frict", 400, NULL, 6, NULL);

    // DMA发送任务
    xTaskCreate(Task_Host, "Task_Host", 500, NULL, 6, NULL);

    // 定义协议发送频率
    Bridge_Send_Protocol(&Node_Host, 0x120, 1);  // 心跳包
    Bridge_Send_Protocol(&Node_Host, 0x403, 20); // 陀螺仪
    // Bridge_Send_Protocol(&Node_Host, 0x404, 10); // 遥控器

    //启动调度,开始执行任务
    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while (1) {
    }
}
