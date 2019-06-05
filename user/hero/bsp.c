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

    // USART
    // BSP_USART2_Init(9600, USART_IT_RXNE);

    // Judge (USART6)
    BSP_USART6_Init(115200, 0);
    BSP_DMA_USART6_RX_Init(Judge.buf, Protocol_Buffer_Length);

    // Ps (USART3)
    BSP_USART3_Init(115200, 0);
    BSP_DMA_USART3_RX_Init(Ps.buf, Protocol_Buffer_Length);

    // Servo
    BSP_PWM_Set_Port(&PWM_Magazine_Servo, PWM_PORT_PD12);
    BSP_PWM_Init(&PWM_Magazine_Servo, 9000, 200, TIM_OCPolarity_Low);
}
