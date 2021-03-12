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

    Motor_Init(&Motor_Yaw, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_Pitch, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_Roll, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Init(&Motor_Stir, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
    Motor_Pitch.position     = 2740;
    Motor_Pitch.positionBias = 2740;
    Motor_Yaw.position       = 3538;
    Motor_Yaw.positionBias   = 3538;

    // 通讯协议初始化
    Protocol_Init(&Node_Judge, &ProtocolData);
    Protocol_Init(&Node_Host, &ProtocolData);
    Protocol_Init(&Node_Board, &ProtocolData);

    // CAN外设
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_Yaw);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_Pitch);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_Roll);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_Stir);
    // 陀螺仪设置静态误差
    Gyroscope_Set_Bias(&ImuData, 0.69, -8, -7.6);

    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);

    // snail 电机pwm输出
    BSP_PWM_Set_Port(&PWM_Snail1, PWM_PD12);
    BSP_PWM_Init(&PWM_Snail1, 180, 1250, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Snail2, PWM_PD13);
    BSP_PWM_Init(&PWM_Snail2, 180, 1250, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Servo, PWM_PH12);
    BSP_PWM_Init(&PWM_Servo, 9000, 200, TIM_OCPolarity_Low);

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

    // 总线设置
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
    TIM5CH1_CAPTURE_STA = 0;
    while (!remoteData.state) {
    }

    // 功能任务
    xTaskCreate(Task_Safe_Mode, "Task_Safe_Mode", 500, NULL, 7, NULL);
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);
    xTaskCreate(Task_Gimbal, "Task_Gimbal", 800, NULL, 5, NULL);
    xTaskCreate(Task_Snail, "Task_Snail", 500, NULL, 6, NULL);
    xTaskCreate(Task_Fire, "Task_Fire", 500, NULL, 7, NULL);
    xTaskCreate(Task_Control, "Task_Control", 500, NULL, 4, NULL);

    // 定义协议发送频率
    // Bridge_Send_Protocol(&Node_Host, 0x120, 1); // 心跳包

    //启动调度,开始执行任务
    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while (1) {
    }
}
