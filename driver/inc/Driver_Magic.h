#ifndef __MAGIC_H
#define __MAGIC_H

#include "stdio.h"
#include "stm32f4xx_conf.h"

typedef struct {
    char *name;
    int   defaultValue; // 默认值
    int   value;        // 调试值
} MagicHandle_Type;

void Magic_Init_Config(u32 bound);                                 // 串口 USART6 初始化 (IO3)
void Magic_Init_Handle(MagicHandle_Type *magic, int defaultValue); // 初始化调试数据的默认值
int  Magic_Get_Debug_Value(MagicHandle_Type *magic);               // 调试数据接收函数

#endif
