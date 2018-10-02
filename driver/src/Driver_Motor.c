/**
 * @brief 电机信息处理
 *
 * @note 转子位置及速度通过CAN接收,在stm32f4xx_it.c中定义
 * @file Driver_Motor.c
 */

#include "Driver_Motor.h"

/**
 * @brief 初始化 Motor
 *
 * @param Motor_Type
 * @param positionBias
 */
void Motor_Init(volatile Motor_Type *motor, int16_t positionBias) {
    motor->position     = 0;
    motor->lastPosition = 0;
    motor->positionBias = positionBias;
    motor->positionDiff = 0;
    motor->speed        = 0;
    motor->round        = 0;
    motor->angle        = 0;
    motor->angleBias    = 0;
    motor->lastAngle    = 0;
}

/**
 * @brief 通过转子位置(position)变化判断电机转过的实际角度(angle)
 *
 * @param Motor_Type
 */
void Motor_Update(volatile Motor_Type *motor) {
    // 更新转子信息
    motor->lastPosition = motor->position;
    motor->positionDiff = motor->position - motor->lastPosition;

    //两次编码器的反馈值差别太大,表示圈数发生了改变
    if (motor->positionDiff < -4200) {
        motor->round++;
    } else if (motor->positionDiff > 4200) {
        motor->round--;
    }

    //计算得到连续角度值,范围正负无穷大
    motor->angle = (float) (motor->position - motor->positionBias) / 8192 * 360 + motor->round * 360;
}
