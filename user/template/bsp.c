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
    BSP_DMA_Init(USART6_Tx, Judge.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(USART6_Rx, Judge.receiveBuf, Protocol_Buffer_Length);
    // BSP_DMA_USART6_RX_Init(Judge.receiveBuf, Protocol_Buffer_Length);
    // BSP_DMA_USART6_TX_Init(Judge.sendBuf, Protocol_Buffer_Length);

    // Board (UART7)
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(UART7_Tx, Board.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(UART7_Rx, Board.receiveBuf, Protocol_Buffer_Length);
    // BSP_DMA_UART7_RX_Init(Board.receiveBuf, Protocol_Buffer_Length);
    // BSP_DMA_UART7_TX_Init(Board.sendBuf, Protocol_Buffer_Length);

    // PS (UART8)
    BSP_UART8_Init(115200, USART_IT_IDLE);
    BSP_DMA_Init(UART8_Tx, Ps.sendBuf, Protocol_Buffer_Length);
    BSP_DMA_Init(UART8_Rx, Ps.receiveBuf, Protocol_Buffer_Length);
    // BSP_DMA_UART8_RX_Init(Ps.receiveBuf, Protocol_Buffer_Length);
    // BSP_DMA_UART8_TX_Init(Ps.sendBuf, Protocol_Buffer_Length);

    // Debug
    // BSP_DMA_USART6_TX_Init(Debug.sendBuf, Protocol_Buffer_Length);
    // BSP_DMA_UART7_TX_Init(Debug.sendBuf, Protocol_Buffer_Length);
    // BSP_DMA_UART8_TX_Init(Debug.sendBuf, Protocol_Buffer_Length);

    // PWM
    // BSP_PWM_Set_Port(&PWM_Test, PWM_PORT_PD12);
    // BSP_PWM_Init(&PWM_Test, 9000, 200, TIM_OCPolarity_Low);
}
