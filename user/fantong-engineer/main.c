#define __HANDLE_GLOBALS
#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"

void BSP_Rescue_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
}

void BSP_LOCK_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void BSP_GO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
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
    // BSP_Rescue_Init();
    BSP_LOCK_Init();
    BSP_GO_Init();

    // USART
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_UART8_Init(115200, USART_IT_IDLE);

    // snail 电机pwm输出
    if (Board_Id == 1) {
        BSP_PWM_Set_Port(&PWM_Snail1, PWM_PD12);
        BSP_PWM_Init(&PWM_Snail1, 180, 1250, TIM_OCPolarity_Low);
        BSP_PWM_Set_Port(&PWM_Snail2, PWM_PD13);
        BSP_PWM_Init(&PWM_Snail2, 180, 1250, TIM_OCPolarity_Low);
    }

    if (Board_Id == 1) {
        // 底盘电机
        Motor_Init(&Motor_LF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
        Motor_Init(&Motor_LB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
        Motor_Init(&Motor_RB, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);
        Motor_Init(&Motor_RF, CHASSIS_MOTOR_REDUCTION_RATE, DISABLE, ENABLE);

        // 云台电机
        Motor_Init(&Motor_Yaw, 1.0, ENABLE, ENABLE);    // 顺时针为正电流
        Motor_Init(&Motor_Pitch, 1.0, ENABLE, DISABLE); // 逆时针为正电流
        Motor_Init(&Motor_Stir, 1.0, ENABLE, ENABLE);   //拨弹
        Motor_Yaw.positionBias   = 1354;
        Motor_Yaw.position       = 1354;
        Motor_Pitch.positionBias = 4766;
        Motor_Pitch.position     = 4766;

        //抬升电机
        Motor_Init(&Motor_Raise_Left, 19.2f, ENABLE, ENABLE);
        Motor_Init(&Motor_Raise_Right, 19.2f, ENABLE, ENABLE);

        // CAN外设
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_LF);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_LB);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_RB);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_RF);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x206, &Motor_Pitch);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x207, &Motor_Raise_Left);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x208, &Motor_Raise_Right);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x209, &Motor_Yaw);
        Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x207, &Motor_Stir);

        // 遥控器数据初始化
        DBUS_Init(&remoteData, &keyboardData, &mouseData);

        // 陀螺仪设置静态误差
        Gyroscope_Set_Bias(&ImuData, 28, 30, 0);
    } else if (Board_Id == 2) {
        // 抓取电机
        Motor_Init(&Motor_Fetch_X, FITCH_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);
        Motor_Init(&Motor_Fetch_Left_Pitch, FITCH_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);
        Motor_Init(&Motor_Fetch_Right_Pitch, FITCH_MOTOR_REDUCTION_RATE, ENABLE, ENABLE);
        Motor_Init(&Motor_Milk, 36.0, ENABLE, ENABLE);

        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_Fetch_X);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_Fetch_Left_Pitch);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_Fetch_Right_Pitch);
        Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_Milk);

        // 陀螺仪设置静态误差
        Gyroscope_Set_Bias(&ImuData, 28, 30, 0);
    }

    // 总线设置
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x501, &Node_Host);
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x502, &Node_Board);
    // Bridge_Bind(&BridgeData, USART_BRIDGE, 6, &Node_Judge);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 7, &Node_Board);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 8, &Node_Host);

    // 陀螺仪
    Gyroscope_Set_Bias(&ImuData, -4, 3, 1); // 设置静态误差
    if (BOARD_CHASSIS) {
        Gyroscope_Init(&Gyroscope_EulerData, 300); // 初始化
    }

    /*******************************************************************************
     *                                 任务初始化                                   *
     *******************************************************************************/

    xTaskCreate(Task_Remote_Share, "Task_Remote_Share", 500, NULL, 9, NULL);

    // 等待遥控器开启
    while (!remoteData.state) {
    }

    // 低级任务
    xTaskCreate(Task_Blink, "Task_Blink", 400, NULL, 3, NULL);
    // xTaskCreate(Task_Startup_Music, "Task_Startup_Music", 400, NULL, 3, NULL);

    //模式切换任务
    xTaskCreate(Task_Control, "Task_Control", 400, NULL, 9, NULL);

    // Can发送任务
    xTaskCreate(Task_Can_Send, "Task_Can_Send", 500, NULL, 5, NULL);

    // 板间通讯
    // xTaskCreate(Task_Board_Communication, "Task_Board_Communication", 500, NULL, 6, NULL);

    // 运动控制任务
    if (BOARD_CHASSIS) {
        xTaskCreate(Task_Chassis, "Task_Chassis", 400, NULL, 5, NULL);
        xTaskCreate(Task_Gimbal, "Task_Gimbal", 500, NULL, 5, NULL);
        xTaskCreate(Task_Raise, "Task_Raise", 400, NULL, 5, NULL); // 抬升
        // xTaskCreate(Task_Rescue, "Task_Rescue", 400, NULL, 5, NULL); // 救援
        xTaskCreate(Task_Fire_Stir, "Task_Fire_Stir", 400, NULL, 5, NULL);
        xTaskCreate(Task_Snail, "Task_Snail", 500, NULL, 5, NULL);
    } else if (BOARD_FETCH) {
        xTaskCreate(Task_Fetch, "Task_Fetch", 400, NULL, 5, NULL);
        xTaskCreate(Task_Milk, "Task_Milk", 400, NULL, 5, NULL);
        xTaskCreate(Task_Go, "Task_Go", 400, NULL, 5, NULL);
    }

    // 定义协议发送频率
    // Bridge_Send_Protocol(&Node_Host, 0x120, 1); // 心跳包

    //启动调度,开始执行任务
    vTaskStartScheduler();

    //系统启动失败:定时器任务或者空闲任务的heap空间不足
    while (1) {
    }
}
