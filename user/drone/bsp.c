/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "Driver_BSP.h"
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

    // snail 电机pwm输出
    BSP_PWM_Set_Port(&PWM_Snail1, PWM_PD12);
    BSP_PWM_Init(&PWM_Snail1, 180, 1250, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Snail2, PWM_PD13);
    BSP_PWM_Init(&PWM_Snail2, 180, 1250, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Servo, PWM_PH12);
    BSP_PWM_Init(&PWM_Servo, 9000, 200, TIM_OCPolarity_Low);

    // Judge (USART6)
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(USART6_Tx, JudgeChannel.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(USART6_Rx, JudgeChannel.receiveBuf, Protocol_Buffer_Length);

    // User (UART7)
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(UART7_Tx, UserChannel.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(UART7_Rx, UserChannel.receiveBuf, Protocol_Buffer_Length);

    // Host (UART8)
    BSP_UART8_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(UART8_Tx, HostChannel.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(UART8_Rx, HostChannel.receiveBuf, Protocol_Buffer_Length);
}
