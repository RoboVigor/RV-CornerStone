#include "Driver_Chassis.h"
#include "macro.h"
#include "config.h"
#include "handle.h"

ChassisParam_Type Chassisparam;

void MoveValueInit(void) {
    //ChassisStatus.NeedStop = 0;
}
/**
 * @brief 配置小车整体 XYW 三个轴的速度
 * @detail
 *
 * @param XSpeed **前后**
 * @param YSpeed **左右**
 * @param WSpeed
 */
void Chassis_Set_Speed(float XSpeed, float YSpeed, float WSpeed) {
    Chassisparam.TargetVX = XSpeed;
    Chassisparam.TargetVY = YSpeed;
    Chassisparam.TargetWR = WSpeed;
}
/*
void Power_Control(void) {
    if (powerfeed >= 60) {
        // ChassisStatus.HasStart = 1;
        ChassisStatus.NeedStop = 0;
    }
    if (Chassisparam.TargetVX == 0 && Chassisparam.TargetVY == 0) {
        // ChassisStatus.HasStart = 0;
        ChassisStatus.NeedStop = 1;
    }
    if (ChassisStatus.NeedStop == 0) {
        // PID_Calculate(&PID_Power, 800, powercurrent * 10);
        PID_Calculate(&PID_Power, 700, powerfeed * 10);
    }

    if (ChassisStatus.NeedStop == 1) {
        PID_Power.output_I = 0;
        PID_Power.output   = 0;
    }

    PowerParam = (float) (PID_Power.output + 1000) / 1000.0;
}*/
/**
 * @brief 麦克纳姆轮解算
 * @detail 电机位置：左上角0,逆时针依次增加
 * @detail 转子的角速度 / 电机减速比 = 轮子角速度 (rad/s)
 *
 * @param result[4] 返回值,轮子速度
 * */
void Chassis_Get_Rotor_Speed(int rotorSpeed[4]) {
    int wheelSpeed[4];
    // 麦克纳姆轮解算
    wheelSpeed[0] = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((Chassisparam.TargetVY) - (Chassisparam.TargetVX) + Chassisparam.TargetWR * -CHASSIS_SIZE_K);
    wheelSpeed[1] = CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((Chassisparam.TargetVY) + (Chassisparam.TargetVX) + Chassisparam.TargetWR * -CHASSIS_SIZE_K);
    wheelSpeed[2] = -CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    (Chassisparam.TargetVY - (Chassisparam.TargetVX) + Chassisparam.TargetWR * CHASSIS_SIZE_K);
    wheelSpeed[3] = -CHASSIS_INVERSE_WHEEL_RADIUS * CHASSIS_MOTOR_REDUCTION_RATE *
                    ((Chassisparam.TargetVY) + (Chassisparam.TargetVX) + Chassisparam.TargetWR * CHASSIS_SIZE_K);
    // 限速
    Chassis_Limit_Rotor_Speed(wheelSpeed, rotorSpeed);
}
/**
 * @brief 按比例限速
 *
 * @param wheelSpeed
 * @param rotorSpeed
 */
void Chassis_Limit_Rotor_Speed(int wheelSpeed[4], int rotorSpeed[4]) {
    float maxSpeed = 0;
    float param    = 0;
    int   index    = 0;
    int   speedLimit;
    int   Buffer[4];

    //speedLimit = (int) (CHASSIS_MAX_ROTOR_SPEED * PowerParam);

    // 打擂台获得麦轮速度最大值
    for (; index < 4; index++) {
        if (ABS(wheelSpeed[index]) > maxSpeed) {
            maxSpeed = ABS(wheelSpeed[index]);
        }
    }
/*
    Buffer[0] = wheelSpeed[0] * PowerParam;
    Buffer[1] = wheelSpeed[1] * PowerParam;
    Buffer[2] = wheelSpeed[2] * PowerParam;
    Buffer[3] = wheelSpeed[3] * PowerParam;
*/
    // 进行限幅
    if (maxSpeed > speedLimit) {
        param         = (float) CHASSIS_MAX_ROTOR_SPEED / maxSpeed;
        rotorSpeed[0] = Buffer[0] * param;
        rotorSpeed[1] = Buffer[1] * param;
        rotorSpeed[2] = Buffer[2] * param;
        rotorSpeed[3] = Buffer[3] * param;
    } else {
        rotorSpeed[0] = Buffer[0];
        rotorSpeed[1] = Buffer[1];
        rotorSpeed[2] = Buffer[2];
        rotorSpeed[3] = Buffer[3];
    }
}