#define __HANDLE_GLOBALS
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"
// #include "queue.h"
void BSP_Proximity_Switch_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
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

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // 硬件配置
    BSP_CAN_Init();
    BSP_DBUS_Init(remoteBuffer);
    BSP_TIM2_Init();
    BSP_IMU_Init();
    BSP_Laser_Init();
    BSP_Beep_Init();
    BSP_LED_Init();
    BSP_User_Power_Init();
    BSP_Proximity_Switch_Init();

    // USART
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_UART8_Init(115200, USART_IT_IDLE);

    // 获取设备ID
    BSP_Stone_Id_Init(&Board_Id, &Robot_Id);

    // 底盘电机
    Motor_Init(&Motor_Chassis_Left, CHASSIS_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);
    Motor_Init(&Motor_Chassis_Right, CHASSIS_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);

    // 云台电机
    Motor_Init(&Motor_Down_Gimbal_Yaw, 36.0f, ENABLE, DISABLE);
    Motor_Init(&Motor_Down_Gimbal_Pitch, 1, ENABLE, ENABLE);

    // 摩擦轮电机
    Motor_Init(&Motor_Down_Frict_Left, 1, ENABLE, DISABLE);
    Motor_Init(&Motor_Down_Frict_Right, 1, ENABLE, DISABLE);

    // 拨弹电机
    Motor_Init(&Motor_Down_Stir, 36.0f, ENABLE, DISABLE);

    // 遥控器数据初始化
    DBUS_Init(&remoteData);

    // 通讯协议初始化
    Protocol_Init(&Node_Judge, &ProtocolData);
    Protocol_Init(&Node_Host, &ProtocolData);
    Protocol_Init(&Node_Board, &ProtocolData);

    // 总线设置
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_Chassis_Left);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_Chassis_Right);

    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_Down_Frict_Left);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_Down_Frict_Right);
    Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x201, &Motor_Down_Stir);
    Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x207, &Motor_Down_Gimbal_Yaw);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x209, &Motor_Down_Gimbal_Pitch);

    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x501, &Node_Host);
    // Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x502, &Node_Board);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 6, &Node_Board);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 7, &Node_Host);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 8, &Node_Judge);

    // 陀螺仪
    Motor_Down_Gimbal_Yaw.positionBias   = 0;
    Motor_Down_Gimbal_Yaw.position       = 0;
    Motor_Down_Gimbal_Pitch.positionBias = 6400;
    Motor_Down_Gimbal_Pitch.position     = 6400;

    Gyroscope_Set_Bias(&ImuData, 10, 27, 13);
    Gyroscope_Init(&Gyroscope_EulerData, 300); // 初始化

    /*******************************************************************************
     *                                 任务初始化                                   *
     *******************************************************************************/

    // 低级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // 板间通信
    // xTaskCreate(Task_Board_Communication, "Task_Board_Communication", 500, NULL, 6, NULL);

    // 等待遥控器开启
    // while (!remoteData.state) {
    // }

    xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);

    xTaskCreate(Task_Down_Gimbal, "Task_Down_Gimbal", 500, NULL, 5, NULL);
    xTaskCreate(Task_Down_Stir, "Task_Down_Stir", 400, NULL, 6, NULL);
    xTaskCreate(Task_Down_Frict, "Task_Down_Frict", 400, NULL, 6, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 6, NULL);

    //启动调度,开始执行任务
    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while (1) {
    }
}
