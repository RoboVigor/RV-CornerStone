/**
 * @file Driver_Motor.h
 * @brief 电机驱动
 * @note 新增电机应在handle.h和handle.c中声明并初始化,在stm32f4xx_it.h中设置对应电调ID
 * @version 0.8
 * - 修正连续角度计算精度问题
 * @version 0.6
 * - 更新 Motor_Init() 参数
 * - 更新 Motor_Update() 参数
 * @version 0.5
 * - Motor_Type 用于保存所有电机相关的信息
 * - 原有的平均值滤波函数将在Driver_Filter中实现
 */

#ifndef __DRIVER_MOTOR_H
#define __DRIVER_MOTOR_H

#include "stm32f4xx.h"

typedef struct {
    int16_t position;     // 转子位置(电调机械角度值), 0-8191
    int16_t lastPosition; // 上一次的转子位置
    int16_t positionBias; // 修正转子位置, 默认(-1)会以开机时转子位置为初始位置
    int16_t positionDiff; // 转子位置差值
    int16_t speed;        // 转子转速值, RPM

    int8_t  angleEnabled; // 连续角度启用
    int16_t round;        // 圈数
    float   angle;        // 转子连续角度
    float   angleBias;    // 修正转子连续角度
    float   lastAngle;    // 上一次的连续角度
    float   reductionRate;
} Motor_Type;

/**
 * @brief 初始化电机结构体
 *
 * @param Motor_Type 电机结构体
 * @param reductionRate 电机减速比(输入转速:输出转速)
 * @param angleEnabled 是否启用连续角度计算
 */
void Motor_Init(volatile Motor_Type *motor, float reductionRate, int8_t angleEnabled);

/**
 * @brief 更新电机数据
 * @note 转子位置及速度通过CAN接收,在stm32f4xx_it.c中定义
 *
 * @param Motor_Type 电机结构体
 * @param position 转子位置
 * @param speed 转子转速
 */
void Motor_Update(volatile Motor_Type *motor, int16_t position, int16_t speed);

#endif
