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

    // USART
    // BSP_USART2_Init(9600, USART_IT_RXNE);

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

    // Servo
    BSP_PWM_Set_Port(&PWM_Magazine_Servo, PWM_PI0);
    BSP_PWM_Init(&PWM_Magazine_Servo, 9000, 200, TIM_OCPolarity_Low);

    // Stone Id
    BSP_Stone_Id_Init(&Board_Id, &Robot_Id);
}
