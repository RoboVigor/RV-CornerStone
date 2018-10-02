#ifndef __DRIVER_MOTOR_H
#define __DRIVER_MOTOR_H

#include "stm32f4xx.h"

#define RATE_BUF_SIZE 6

typedef struct {
    int16_t position;     // 转子位置(电调机械角度值), 0-8191
    int16_t lastPosition; // 上一次的转子位置
    int16_t positionBias; // 初始转子位置
    int16_t positionDiff; // 转子位置差值
    int16_t speed;        // 转子转速值, RPM
    int16_t round;        // 圈数
    float   angle;        // 转子连续角度
    float   lastAngle;    // 上一次的连续角度

    // todo:新建滤波算法Driver
    // uint8_t bufCount;               //滤波更新buf用
    // int16_t ecdRawRate;             //通过编码器计算得到的速度原始值
    // int16_t rateBuf[RATE_BUF_SIZE]; // buf,for filter
    // int16_t filterRate;             //速度
} Motor_Type;

void Motor_Init(volatile Motor_Type *motor, int16_t positionBias);
void Motor_Update(volatile Motor_Type *motor);

#endif
