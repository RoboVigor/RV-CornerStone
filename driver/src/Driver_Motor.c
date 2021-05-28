#include "Driver_Motor.h"
#include "macro.h"

#define PI 3.1415926

void Motor_Init(volatile Motor_Type *motor, float reductionRate, int8_t angleEnabled, int8_t inputEnabled) {
    motor->positionBias  = -1; // -1为未赋值状态
    motor->reductionRate = reductionRate;
    motor->angleEnabled  = angleEnabled;
    motor->inputEnabled  = inputEnabled;
}

void Motor_Update(volatile Motor_Type *motor, uint8_t data[8]) {
    // 数据解包
    int16_t position      = data[0] << 8 | data[1];
    int16_t speed         = data[2] << 8 | data[3];
    int16_t actualCurrent = data[4] << 8 | data[5];
    int16_t temperature   = data[6];
    motor->updateAt       = xTaskGetTickCountFromISR();

    // 更新转子初始位置
    if (motor->positionBias == -1) {
        motor->positionBias = position;
        motor->position     = position;
        return;
    }

    // 更新转子信息
    motor->lastPosition  = motor->position;
    motor->position      = position;
    motor->speed         = speed;
    motor->actualCurrent = (actualCurrent / 16384.0) * 20;
    motor->torque        = MAX(24 * motor->actualCurrent, 0) / (speed * PI / 30.0 / motor->reductionRate + 2);
    motor->temperature   = temperature;

    //如果启用了连续角度计算
    if (motor->angleEnabled) {
        //两次编码器的反馈值差别太大,表示圈数发生了改变
        motor->positionDiff = motor->position - motor->lastPosition;
        if (motor->positionDiff < -4200) {
            motor->round++;
        } else if (motor->positionDiff > 4200) {
            motor->round--;
        }
        //计算得到连续角度值,范围正负无穷大
        motor->angle = ((float) (motor->position - motor->positionBias) / 8192.0f * 360 + motor->round * 360) / motor->reductionRate;
    }
}
