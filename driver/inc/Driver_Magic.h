#ifndef __MAGIC_H
#define __MAGIC_H

#include "stdio.h"
#include "stm32f4xx_conf.h"

#define MAGIC_MAX_LENGTH 200 //定义最大接收字节数 200

typedef struct {
    char *name;
    int   defaultValue; // 默认值
    int   value;        // 调试值
} MagicHandle_Type;

/**
 * @brief 串口 USART6 初始化
 *
 * @param bound 波特率
 */
void Magic_Init_Config(u32 bound);

/**
 * @brief 初始化调试数据的默认值
 *
 * @param magic 无线串口数据结构体
 * @param defaultValue 默认值
 */
void Magic_Init_Handle(MagicHandle_Type *magic, int defaultValue);

/**
 * @brief 调试数据接收函数
 */
void Magic_Get_Debug_Value(MagicHandle_Type *magic);

#endif
