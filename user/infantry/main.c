#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"
// #include "queue.h"

void BSP_Init(void) {
    BSP_CAN_Init();
    BSP_DBUS_Init(remoteBuffer);
    BSP_TIM2_Init();
    BSP_IMU_Init();
    BSP_Laser_Init();
    BSP_Beep_Init();
    BSP_LED_Init();
    BSP_User_Power_Init();

    // USART
    // BSP_USART2_Init(9600, USART_IT_RXNE);

    // Judge (USART6)
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(USART6_Tx, Node_Judge.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(USART6_Rx, Node_Judge.receiveBuf, Protocol_Buffer_Length);

    // User (UART7)
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(UART7_Tx, Node_Board.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(UART7_Rx, Node_Board.receiveBuf, Protocol_Buffer_Length);

    // Host (UART8)
    BSP_UART8_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(UART8_Tx, Node_Host.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(UART8_Rx, Node_Host.receiveBuf, Protocol_Buffer_Length);

    // Servo
    BSP_PWM_Set_Port(&PWM_Magazine_Servo, PWM_PI0);
    BSP_PWM_Init(&PWM_Magazine_Servo, 9000, 200, TIM_OCPolarity_Low);

    // Stone Id
    BSP_Stone_Id_Init(&Board_Id, &Robot_Id);
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

    // 底盘电机
    Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

    // 发射机构电机
    Motor_Init(&Motor_Stir, STIR_MOTOR_REDUCTION_RATE, ENABLE, ENABLE); //拨弹
    Motor_Init(&Motor_FL, FIRE_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_FR, FIRE_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

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

    // USART
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_UART8_Init(115200, USART_IT_IDLE);

    // Servo
    BSP_PWM_Set_Port(&PWM_Magazine_Servo, PWM_PI0);
    BSP_PWM_Init(&PWM_Magazine_Servo, 9000, 200, TIM_OCPolarity_Low);

    if (Robot_Id == 1) {
        Motor_Yaw.positionBias   = 4110;
        Motor_Yaw.position       = 4110;
        Motor_Pitch.positionBias = 5540;
        Motor_Pitch.position     = 5540;
        Gyroscope_Set_Bias(&ImuData, 3, 15, 46);
    } else if (Robot_Id == 2) {
        Motor_Yaw.positionBias   = 3400;
        Motor_Yaw.position       = 3400;
        Motor_Pitch.positionBias = 7090;
        Motor_Pitch.position     = 7090;
        Gyroscope_Set_Bias(&ImuData, 27, -2, 12);
    }

    // 总线设置
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_LF);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_LB);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_RB);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_RF);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x206, &Motor_Pitch);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x209, &Motor_Yaw);
    Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x201, &Motor_FL);
    Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x202, &Motor_FR);
    Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x207, &Motor_Stir);

    // 总线设置
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x501, &Node_Host);
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x502, &Node_Board);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 6, &Node_Judge);
    // Bridge_Bind(&BridgeData, USART_BRIDGE, 7, &Node_Board);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 8, &Node_Host);

    // 陀螺仪
    Gyroscope_Init(&Gyroscope_EulerData,300); // 初始化

    /*******************************************************************************
     *                                 任务初始化                                   *
     *******************************************************************************/

    // 等待遥控器开启
    while (!remoteData.state) {
    }

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
    // xTaskCreate(Task_Board_Communication, "Task_Board_Communication", 500, NULL, 6, NULL);
    // xTaskCreate(Task_Vision_Communication, "Task_Vision_Communication", 500, NULL, 6, NULL);

    
    //启动调度,开始执行任务
    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while (1) {
    }
}
