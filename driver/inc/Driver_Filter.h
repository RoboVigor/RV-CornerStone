#ifndef __Filter_H
#define __Filter_H

#include "stm32f4xx_conf.h"

typedef struct {
    int8_t isInit;      // 自动初始化
    float  thresholdLB; // 限幅滤波阈值

    float value;
    float lastValue;
    float diff;
    float offset;
    float result;
} Filter_Type;

/**
 * @brief 更新基础滤波器
 */
void Filter_Update(Filter_Type *filter, float value);

/**
 * @brief 应用限幅滤波
 */
float Filter_Limit_Breadth(Filter_Type *filter);

#endif
