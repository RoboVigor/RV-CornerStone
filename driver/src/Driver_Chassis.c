#include "Driver_Chassis.h"
#include "macro.h"
#include "config.h"

/**
 * @brief 配置小车整体 XYW 三个轴的速度
 * @detail
 *
 * @param XSpeed **前后**
 * @param YSpeed **左右**
 * @param WSpeed
 */
void Chassis_Update(ChassisData_Type *ChassisData, float XSpeed, float YSpeed, float WSpeed) {
    ChassisData->TargetVX = XSpeed;
    ChassisData->TargetVY = YSpeed;
    ChassisData->TargetWR = WSpeed;
}

/**
 * @brief 麦克纳姆轮解算
 * @detail 电机位置：左上角0,逆时针依次增加
 * @detail 转子的角速度 / 电机减速比 = 轮子角速度 (rad/s)
 *
 * @param result[4] 返回值,轮子速度
 * */
void Chassis_Get_Rotor_Speed(ChassisData_Type *ChassisData, int rotorSpeed[4]) {
    int wheelSpeed[4];
    // 麦克纳姆轮解算
    wheelSpeed[0] = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((ChassisData->TargetVY) - (ChassisData->TargetVX) + ChassisData->TargetWR * -CHASSIS_SIZE_K);
    wheelSpeed[1] = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((ChassisData->TargetVY) + (ChassisData->TargetVX) + ChassisData->TargetWR * -CHASSIS_SIZE_K);
    wheelSpeed[2] = -CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    (ChassisData->TargetVY - (ChassisData->TargetVX) + ChassisData->TargetWR * CHASSIS_SIZE_K);
    wheelSpeed[3] = -CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((ChassisData->TargetVY) + (ChassisData->TargetVX) + ChassisData->TargetWR * CHASSIS_SIZE_K);
    // 限速
    Chassis_Limit_Rotor_Speed(&ChassisData, wheelSpeed, rotorSpeed);
}
/**
 * @brief 按比例限速
 *
 * @param wheelSpeed
 * @param rotorSpeed
 */
void Chassis_Limit_Rotor_Speed(ChassisData_Type *ChassisData, int wheelSpeed[4], int rotorSpeed[4]) {
    float maxSpeed = 0;
    float param    = 0;
    int   index    = 0;

    // 打擂台获得麦轮速度最大值
    for (; index < 4; index++) {
        if (ABS(wheelSpeed[index]) > maxSpeed) {
            maxSpeed = ABS(wheelSpeed[index]);
        }
    }

    // 进行限幅
    if (maxSpeed > CHASSIS_MAX_ROTOR_SPEED) {
        param         = (float) CHASSIS_MAX_ROTOR_SPEED / maxSpeed;
        rotorSpeed[0] = wheelSpeed[0] * param;
        rotorSpeed[1] = wheelSpeed[1] * param;
        rotorSpeed[2] = wheelSpeed[2] * param;
        rotorSpeed[3] = wheelSpeed[3] * param;
    } else {
        rotorSpeed[0] = wheelSpeed[0];
        rotorSpeed[1] = wheelSpeed[1];
        rotorSpeed[2] = wheelSpeed[2];
        rotorSpeed[3] = wheelSpeed[3];
    }
}
