/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "handle.h"

void BSP_Capacitor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    //  charge/discharge
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    //  read  voltage/current
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_2;
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

    // USART
    // BSP_USART2_Init(9600, USART_IT_RXNE);

#ifdef ROBOT_LOOP_ONE
    // Ps (UART8)
    BSP_UART8_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(UART8_Tx, Ps.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(UART8_Rx, Ps.receiveBuf, Protocol_Buffer_Length);
#endif

#ifdef ROBOT_LOOP_TWO
    // Ps (USART3)
    BSP_USART3_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(USART3_Tx, Ps.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(USART3_Rx, Ps.receiveBuf, Protocol_Buffer_Length);
#endif

#ifdef ROBOT_LOOP_THREE
    // Ps (USART3)
    BSP_USART3_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(USART3_Tx, Ps.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(USART3_Rx, Ps.receiveBuf, Protocol_Buffer_Length);
#endif

    // Judge (USART6)
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(USART6_Tx, Judge.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(USART6_Rx, Judge.receiveBuf, Protocol_Buffer_Length);

    //  ADC
    BSP_ADC1_Init(ADC_CHANNEL_NUM, ADC_Channel1 | ADC_Channel2, 0);
    BSP_DMA_Init(ADC1_Rx, ADC_Value, ADC_BUFFER_LENGTH);
    
    // Servo
    BSP_PWM_Set_Port(&PWM_Magazine_Servo, PWM_PI0);
    BSP_PWM_Init(&PWM_Magazine_Servo, 9000, 200, TIM_OCPolarity_Low);

    // snail 电机pwm输出
    // BSP_PWM_Set_Port(&PWM_Snail1, PWM_PD12);
    // BSP_PWM_Init(&PWM_Snail1, 180, 1250, TIM_OCPolarity_Low);
    // BSP_PWM_Set_Port(&PWM_Snail2, PWM_PD13);
    // BSP_PWM_Init(&PWM_Snail2, 180, 1250, TIM_OCPolarity_Low);
}
