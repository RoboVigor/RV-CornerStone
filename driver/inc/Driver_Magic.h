#ifndef __MAGIC_H
#define __MAGIC_H

#include "stdio.h"
#include "stm32f4xx_conf.h"

#define MAGIC_MAX_LENGTH 200 //定义最大接收字节数 200

typedef struct {
    uint8_t  buf[MAGIC_MAX_LENGTH];
    uint16_t sta;

    char *name;
    int   defaultValue; // 默认值
    int   value;        // 调试值
} MagicHandle_Type;

typedef struct {
    int16_t debug1;
    int16_t debug2;
    int16_t debug3;
    int16_t debug4;
    int16_t debug5;
    int16_t debug6;
    int16_t debug7;
    int16_t debug8;
} DebugData_Type;

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
