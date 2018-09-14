#ifndef __MAGIC_H
#define __MAGIC_H

#include "stdio.h"
#include "stm32f4xx_conf.h"

typedef struct {
    char *name;
    int   defaultValue; // 默认值
    int   value;        // 调试值
} MagicHandle_Type;

void Magic_Init_Config(u32 bound);
void Magic_Init_Handle(MagicHandle_Type *magic, int defaultValue);
int  Magic_Get_Debug_Value(MagicHandle_Type *magic);

#endif
