/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "handle.h"

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

void BSP_Init(void) {
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
    // BSP_USART2_Init(9600, USART_IT_RXNE);

    // Judge (USART6)
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(USART6_Tx, JudgeChannel.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(USART6_Rx, JudgeChannel.receiveBuf, Protocol_Buffer_Length);

    // Board (UART7)
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(UART7_Tx, UserChannel.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(UART7_Rx, UserChannel.receiveBuf, Protocol_Buffer_Length);

    // PS (UART8)
    // BSP_UART8_Init(115200, USART_IT_IDLE);
    // BSP_DMA_Init(UART8_Tx, HostChannel.sendBuf, Protocol_Buffer_Length);
    // BSP_DMA_Init(UART8_Rx, HostChannel.receiveBuf, Protocol_Buffer_Length);

    // PWM
    // BSP_PWM_Set_Port(&PWM_Test, PWM_PD12);
    // BSP_PWM_Init(&PWM_Test, 9000, 200, TIM_OCPolarity_Low);

    BSP_PWM_Set_Port(&PWM_Hook_L, PWM_PA1);
    BSP_PWM_Init(&PWM_Hook_L, 9000, 200, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Hook_R, PWM_PA2);
    BSP_PWM_Init(&PWM_Hook_R, 9000, 200, TIM_OCPolarity_Low);
}