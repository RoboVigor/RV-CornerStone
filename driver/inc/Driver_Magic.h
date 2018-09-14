#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "stm32f4xx_conf.h"

#define USART_REC_LEN 200 //定义最大接收字节数 200
#define EN_USART6_RX 1    //使能(1)/禁止(0)串口3接收

typedef USART_InitTypeDef Magic_InitTypedef;

typedef struct {
    int defaultNumber = 0;  // 默认值
    int debugNumber;    // 调试值
}MagicTypedef;

int USART_Last_Debug_Number    = -1;
int USART_Default_Debug_Number = 0;

void Magic_Set_Default_Number(int defaultNumber);
int  Magic_Get_Debug_Number(void);

#endif