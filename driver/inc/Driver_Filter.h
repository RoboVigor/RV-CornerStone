/**
 * @file Driver_Filter.h
 * @brief 滤波算法
 * @note "应用"函数会把结果保存到滤波器的result中
 * @version 0.7
 * - 原isInit改为count
 * - 新增采样函数
 * @version 0.6
 * - 新增基础滤波器
 * - 新增限幅滤波
 */
#ifndef __DRIVER_FILTER_H
#define __DRIVER_FILTER_H

#include "stm32f4xx_conf.h"

typedef struct {
    // 基本滤波器
    float   value;
    float   lastValue;
    float   diff;
    float   offset;
    float   result;
    int32_t count; // 计数器

    // 限幅滤波阈值
    float thresholdLB;

    // 采样
    float average; // 移动平均值
    float max;     // 最大值
    float min;     // 最小值

    // 移动平均值
    int16_t windowSize;
    float   movingAverage;
    float * movingAverageArray;
} Filter_Type;

/**
 * @brief 计算基础滤波器
 */
void Filter_Update(Filter_Type *filter, float value);

/**
 * @brief 计算移动平均值
 */
void Filter_Update_Sample(Filter_Type *filter);

/**
 * @brief 计算移动平均值
 */
void Filter_Update_Moving_Average(Filter_Type *filter);

/**
 * @brief 应用限幅滤波
 */
float Filter_Apply_Limit_Breadth(Filter_Type *filter);

#endif
