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
    int16_t  position;      // 转子位置(电调机械角度值), 0-8191
    int16_t  lastPosition;  // 上一次的转子位置
    int16_t  positionDiff;  // 转子位置差值
    int16_t  speed;         // 转子转速值, RPM
    float    actualCurrent; //电流
    int16_t  temperature;   //温度
    uint16_t updatedAt;     // 更新电机数据时的tickCount
    uint8_t  online;        // 电机是否离线

    int8_t  angleEnabled;  // 连续角度启用
    int16_t round;         // 圈数
    float   angle;         // 转子连续角度
    int16_t positionBias;  // 修正转子位置 (目前仅为向下兼容而保留)
    int16_t angleBias;     // 修正转子连续角度, 默认(-1)会以开机时转子位置为初始位置
    int16_t angleBiasInit; // 是否已修正转子连续角度
    float   reductionRate; // 减速比
    float   torque;        // 转矩

    int8_t  inputEnabled; // 输入启用
    int16_t input;        // 输入电流
} Motor_Type;

/**
 * @brief 初始化电机结构体
 *
 * @param Motor_Type 电机结构体
 * @param reductionRate 电机减速比(输入转速:输出转速)
 * @param angleEnabled 是否启用连续角度计算
 * @param inputEnabled 是否启用电流输出
 */
void Motor_Init(Motor_Type *motor, float reductionRate, int8_t angleEnabled, int8_t inputEnabled);

/**
 * @brief 更新电机数据
 * @note 转子位置及速度通过CAN接收,在stm32f4xx_it.c中定义
 *
 * @param Motor_Type 电机结构体
 * @param position 转子位置
 * @param speed 转子转速
 */
void Motor_Update(Motor_Type *motor, uint8_t data[8]);

/**
 * @brief 设置连续角度偏移量
 *
 * @param angleBias 需要设置的偏移量
 */
void Motor_Set_Angle_Bias(Motor_Type *motor, float angleBias);

#endif
