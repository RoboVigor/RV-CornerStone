#ifndef DRIVER_FIRE_H
#define DRIVER_FIRE_H

#include "stm32f4xx.h"

#define LASER_ON GPIO_SetBits(GPIOG, GPIO_Pin_13)    // 激光开启
#define LASER_OFF GPIO_ResetBits(GPIOG, GPIO_Pin_13) // 激光关闭

/**
 * @brief 发射机构状态标志位
 *
 */
typedef struct {
    uint8_t State_Frict; // 摩擦轮 0 停止 1 转动
    uint8_t State_Stir;  // 拨弹轮 0 静止 1 单发 2 连续
} FireState_Type;

/**
 * @brief 初始化发射机构状态标志位
 *
 * @param FireState
 */
void Fire_StateInit(FireState_Type *FireState);

#endif