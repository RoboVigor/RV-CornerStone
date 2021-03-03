/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "handle.h"

void BSP_Init(void) {
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
    // BSP_USART6_Init(115200, USART_IT_IDLE);
    // BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_UART8_Init(115200, USART_IT_IDLE);

    // PWM
    // BSP_PWM_Set_Port(&PWM_Test, PWM_PD12);
    // BSP_PWM_Init(&PWM_Test, 9000, 200, TIM_OCPolarity_Low);

    // ADC
    BSP_ADC1_Init(ADC_CHANNEL_NUM, ADC_Channel6, 0);

    // 总线设置
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x201, &Motor_LF);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x202, &Motor_LB);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x203, &Motor_RB);
    Bridge_Bind(&BridgeData, CAN1_BRIDGE, 0x204, &Motor_RF);
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x501, &HostChannel);
    // Bridge_Bind(&BridgeData, CAN2_BRIDGE, 0x502, &UserChannel);
    // Bridge_Bind(&BridgeData, USART_BRIDGE, 6, &JudgeChannel);
    // Bridge_Bind(&BridgeData, USART_BRIDGE, 7, &UserChannel);
    Bridge_Bind(&BridgeData, USART_BRIDGE, 8, &HostChannel);
}
