#include "BSP_UART.h"

/**
 * @brief  UART初始化
 * @param  void
 * @return void
 * USART6 用作无线串口,定义在usart.c中
 */
void BSP_UART_Init(void) {
    USART_InitTypeDef USART_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    // UART1(DBus)
    USART_InitStructure.USART_BaudRate            = 100000;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    USART_Cmd(USART1, ENABLE);
}
