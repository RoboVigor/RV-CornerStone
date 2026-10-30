#define __HANDLE_GLOBALS

#include "config.h"
#include "macro.h"
#include "handle.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tasks.h"
#include "gimbal_tuning.h"

void PF0_3V3(){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOF,&GPIO_InitStructure);
    GPIO_SetBits(GPIOF,GPIO_Pin_0);
}

int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    //初始化
    //拉起一个3.3v的供电脚
    PF0_3V3();
    Delay_Init(180);
    //机械臂USART6
    BSP_USART6_Init(4000000,USART_IT_IDLE);
    // USART3 is exclusively owned by the tuning protocol in tuning builds.
    BSP_USART3_Init(500000,0);
#if !GIMBAL_TUNING_ENABLED
    Vofa_Debug_Init(&vf_debug,3,3);
#endif
    //CAN总线
    BSP_CAN_Init();

    //6020关节0（两个6020组成的电机组）初始化
    Motor_Init(&Motor6020_MecArm_0_0,GM6020,1,MOTOR_TYPE_GM6020,2.0f,1,1);
    Motor_Set_Angle_Bias(&Motor6020_MecArm_0_0,30.5639648f);
    Motor_Init(&Motor6020_MecArm_0_1,GM6020,1,MOTOR_TYPE_GM6020,2.0f,1,1);
    Motor_Set_Angle_Bias(&Motor6020_MecArm_0_1,146.821289f);
    PID_Init(&Motor6020_MecArm_Group_0_PosPID, 1.8f, 0.0f, 25.0f, 2.0f, 1.0f);
    PID_Init(&Motor6020_MecArm_Group_0_SpeedPID, 800.0f, 16.0f, 0.0f, 16384.0f, 10000.0f);
    Motor6020_MecArm_Group_0.motor0=&Motor6020_MecArm_0_0;
    Motor6020_MecArm_Group_0.motor1=&Motor6020_MecArm_0_1;
    Bridge_Bind(&BridgeData,CAN1_BRIDGE,0x205,&Motor6020_MecArm_0_0);
    Bridge_Bind(&BridgeData,CAN1_BRIDGE,0x206,&Motor6020_MecArm_0_1);


    //宇树电机0，1初始化
    Unitree_Motor_Init(&Unitree_MecArm_1,0,6.33f,1,-1.48787534f);
    Unitree_Motor_Init(&Unitree_MecArm_2,1,6.33f,1,-0.627314031f);
    //宇树电机桥初始化
    Unitree_Bridge_Init(&Unitree_Bridge,USART6,6);
    //宇树电机绑定宇树电机桥
    Unitree_Bridge_Bind(&Unitree_Bridge,&Unitree_MecArm_1);
    Unitree_Bridge_Bind(&Unitree_Bridge,&Unitree_MecArm_2);

    //6020关节三初始化
    Motor_Init(&Motor6020_MecArm_3,GM6020,1,MOTOR_TYPE_GM6020,2.444444f,1,1);
    Motor_Set_Angle_Bias(&Motor6020_MecArm_3,55.7306595f);
    PID_Init(&Motor6020_MecArm_3_PosPID, 8.0f, 0.0f, -0.2f, 10.0f, 1.0f);
    PID_Init(&Motor6020_MecArm_3_SpeedPID, 180.0f, 20.0f, 0.0f, 16384.0f, 10000.0f);
    Bridge_Bind(&BridgeData,CAN1_BRIDGE,0x208,&Motor6020_MecArm_3);

    //关节0初始化
    Arm_Joint_Init(&Arm_Joint_0,
        JOINT_ACTUATOR_DJI_CAN,
        &Motor6020_MecArm_Group_0,0.0f,0.0f,0.0f,0.0f,
        Motor6020_Group_Set_Target,
        Motor6020_Group_Get_Angle,
        Motor6020_Group_Get_Speed,
    Motor6020_Group_Is_Online);

    //关节1初始化
    Arm_Joint_Init(&Arm_Joint_1,
        JOINT_ACTUATOR_UNITREE_UART,
        &Unitree_MecArm_1,-1.59295785f,1.5306778f,0.0f,0.0f,
        Unitree_Set_Target,
        Unitree_Get_Angle,
        Unitree_Get_Speed,
    Unitree_Is_Online);

    //关节2初始化
    Arm_Joint_Init(&Arm_Joint_2,
        JOINT_ACTUATOR_UNITREE_UART,
        &Unitree_MecArm_2,-1.86728072f,2.69006801f,0.0f,0.0f,
        Unitree_Set_Target,
        Unitree_Get_Angle,
        Unitree_Get_Speed,
    Unitree_Is_Online);
    
    //关节3初始化
    Arm_Joint_Init(&Arm_Joint_3,
        JOINT_ACTUATOR_DJI_CAN,
        &Motor6020_MecArm_3,-1.65461532f,1.7253028135f,0.0f,0.0f,
        Motor6020_Set_Target
        ,Motor6020_Get_Angle
        ,Motor6020_Get_Speed
    ,Motor6020_Is_Online);

    MecArm_Init(&MecArm,
        &Arm_Joint_0,
        &Arm_Joint_1,
        &Arm_Joint_2,
        &Arm_Joint_3,
        &Arm_Joint_4,
    5,1);

    // Must initialize before any control task; no implicit joint0 enable.
    GimbalTuning_Init();

    // 延时等待电机上电启动稳定
    delay_ms(1500);

    //创建机械臂任务
    xTaskCreate(
        Task_MecArm,
        "Task_MecArm",
        1024, // words: bounded protocol buffers/call depth; verify high-water on target
        NULL,
        5,
        &MecArmTask_Handler
    );

    vTaskStartScheduler();
    
    while (1){

    }
}
