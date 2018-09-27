#include "Driver_PanController.h"
#include "macro.h"
#include "config.h"
#include "handle.h"

ChassisParam_Type ChassisParam;

/**
 * @brief 配置小车整体 XYZ 三个轴的速度
 * @detail
 *
 * @param XSpeed
 * @param YSpeed
 * @param WSpeed
 */
void Chassis_Set_Speed(int XSpeed, int YSpeed, int WSpeed) {
    ChassisParam.TargetVX = (float) XSpeed / 660 * 1;
    ChassisParam.TargetVY = (float) YSpeed / 660 * 1;
    ChassisParam.TargetWR = -(float) WSpeed / 660 * 0.5; // 4*1.5*2
}

/**
 * @brief 麦克纳姆轮解算
 * @detail 电机位置：左上角 0 ，逆时针，依次增加
 * @detail 转子的角速度 / 电机减速比 = 轮子角速度 (rad/s)
 *
 * @param result[4] 返回值,轮子速度
 * */
void Chassis_Update_Wheel_Speed(int result[4]) {
    int wheelSpeed[4];
    // 麦克纳姆轮解算
    wheelSpeed[0] = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((ChassisParam.TargetVX) - (ChassisParam.TargetVY) + ChassisParam.TargetWR * -CHASSIS_SIZE_K);
    wheelSpeed[1] = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((ChassisParam.TargetVX) + (ChassisParam.TargetVY) + ChassisParam.TargetWR * -CHASSIS_SIZE_K);
    wheelSpeed[2] = -CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    (ChassisParam.TargetVX - (ChassisParam.TargetVY) + ChassisParam.TargetWR * CHASSIS_SIZE_K);
    wheelSpeed[3] = -CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((ChassisParam.TargetVX) + (ChassisParam.TargetVY) + ChassisParam.TargetWR * CHASSIS_SIZE_K);
    // 限速
    Chassis_Limit_Wheel_Speed(wheelSpeed, result, CHASSIS_MAX_WHEEL_SPEED);
}
/**
 * @brief 按比例限速
 * @note 需要评估
 *
 * @param WheelSpeedOrigin
 * @param WheelSpeedRes
 * @param MaxWheelSpeed
 */
void Chassis_Limit_Wheel_Speed(int WheelSpeedOrigin[4], int WheelSpeedRes[4], int MaxWheelSpeed) {
    float MaxSpeed = 0;
    float Param    = 0;
    int   index    = 0;

    for (; index < 4; index++) {
        if (ABS(WheelSpeedOrigin[index]) > MaxSpeed) {
            MaxSpeed = ABS(WheelSpeedOrigin[index]);
        }
    }

    if (CHASSIS_MAX_WHEEL_SPEED < MaxSpeed) {
        Param            = (float) CHASSIS_MAX_WHEEL_SPEED / MaxSpeed;
        WheelSpeedRes[0] = WheelSpeedOrigin[0] * Param;
        WheelSpeedRes[1] = WheelSpeedOrigin[1] * Param;
        WheelSpeedRes[2] = WheelSpeedOrigin[2] * Param;
        WheelSpeedRes[3] = WheelSpeedOrigin[3] * Param;
    } else {
        WheelSpeedRes[0] = WheelSpeedOrigin[0];
        WheelSpeedRes[1] = WheelSpeedOrigin[1];
        WheelSpeedRes[2] = WheelSpeedOrigin[2];
        WheelSpeedRes[3] = WheelSpeedOrigin[3];
    }
}
