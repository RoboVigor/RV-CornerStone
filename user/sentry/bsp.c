/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "handle.h"

void BSP_Optoelectronic_Input(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
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
    BSP_Optoelectronic_Input();

    // snail 电机pwm输出
    BSP_PWM_Set_Port(&PWM_Snail1, PWM_PORT_PD12);
    BSP_PWM_Init(&PWM_Snail1, 180, 1250, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Snail2, PWM_PORT_PD13);
    BSP_PWM_Init(&PWM_Snail2, 180, 1250, TIM_OCPolarity_Low);

    // USART
    // BSP_USART2_Init(9600, USART_IT_RXNE);

    // Judge (USART6)
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_DMA_USART6_RX_Init(Judge.receiveBuf, Protocol_Buffer_Length);
    BSP_DMA_USART6_TX_Init(Judge.sendBuf, Protocol_Buffer_Length);

    // Ps (USART3)
    BSP_USART3_Init(115200, USART_IT_IDLE);
    BSP_DMA_USART3_RX_Init(Ps.receiveBuf, Protocol_Buffer_Length);
    BSP_DMA_USART3_TX_Init(Ps.sendBuf, Protocol_Buffer_Length);
}
