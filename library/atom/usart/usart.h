#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"

#define USART_REC_LEN 200 //定义最大接收字节数 200
#define EN_USART6_RX 1    //使能(1)/禁止(0)串口3接收

// #ifdef __SYSTEM_USART_GLOBALS
// #define __SYSTEM_USART_EXT
// #else
// #define __SYSTEM_USART_EXT extern
// #endif

// __SYSTEM_USART_EXT u8 USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
// __SYSTEM_USART_EXT u16 USART_RX_STA;               //接收状态标记

void uart_init(u32 bound);
#endif
