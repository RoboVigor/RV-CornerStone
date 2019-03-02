#include "BSP_UART.h"

/**
 * @brief  UART初始化
 * @param  void
 * @return void
 * USART6 用作无线串口,定义在usart.c中
 */
void BSP_UART_Init(void) {
    USART_InitTypeDef USART_InitStructure;

    // USART1(DBus)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    USART_InitStructure.USART_BaudRate            = 100000;                         //波特率设置
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件数据流控制
    USART_InitStructure.USART_Mode                = USART_Mode_Rx;                  //收发模式
    USART_InitStructure.USART_Parity              = USART_Parity_No;                //奇偶校验位
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;               //停止位
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;            //字长
    USART_Init(USART1, &USART_InitStructure);                                       //初始化串口
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);                                  // DMA设置
    USART_Cmd(USART1, ENABLE);                                                      //使能串口

    // USART3
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);                          //使能USART3时钟
    USART_InitStructure.USART_BaudRate            = 9600;                           //波特率设置
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;            //字长为8位数据格式
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;               //一个停止位
    USART_InitStructure.USART_Parity              = USART_Parity_No;                //无奇偶校验位
    USART_Init(USART3, &USART_InitStructure);                                       //初始化串口
    USART_Cmd(USART3, ENABLE);                                                      //使能串口
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                  //开启相关中断

    // USART6
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);                          //使能USART6时钟
    USART_InitStructure.USART_BaudRate            = 9600;                           //波特率设置
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;            //字长为8位数据格式
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;               //一个停止位
    USART_InitStructure.USART_Parity              = USART_Parity_No;                //无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;  //收发模式
    USART_Init(USART6, &USART_InitStructure);                                       //初始化串口
    USART_Cmd(USART6, ENABLE);                                                      //使能串口
    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);                                  //开启相关中断
}
