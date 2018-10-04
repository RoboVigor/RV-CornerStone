#ifndef __DRIVER_MOTOR_H
#define __DRIVER_MOTOR_H

#include "stm32f4xx.h"

typedef struct {
    int16_t position;     // 转子位置(电调机械角度值), 0-8191
    int16_t lastPosition; // 上一次的转子位置
    int16_t positionBias; // 修正转子位置
    int16_t positionDiff; // 转子位置差值
    int16_t speed;        // 转子转速值, RPM

    int8_t  angleEnabled; // 连续角度启用
    int16_t round;        // 圈数
    float   angle;        // 转子连续角度
    float   angleBias;    // 修正转子连续角度
    float   lastAngle;    // 上一次的连续角度
    float   reductionRate;
} Motor_Type;

void Motor_Init(volatile Motor_Type *motor, float reductionRate, int8_t angleEnabled);
void Motor_Update(volatile Motor_Type *motor, int16_t position, int16_t speed);

#endif
