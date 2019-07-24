/**
 * @file Driver_Chassis.h
 * @brief 底盘运动驱动
 */

#ifndef __DRIVER_CHASSIS_H
#define __DRIVER_CHASSIS_H

#include "stm32f4xx.h"
#include "Driver_PID.h"

typedef struct {
    // 转子转速
    int16_t rotorSpeed[4];
    float   vx;
    float   vy;
    float   vw;
    // 功率限制
    PID_Type PID_Power;
    float    power;
    float    maxPower;
    float    targetPower;
    float    referencePower;
    float    lastReferencePower;
    float    interval;
    uint32_t fittingCounter;
    float    powerScale;
    float    powerBuffer;
    float    maxPowerBuffer;
} ChassisData_Type;

/**
 * @brief 初始化底盘
 */
void Chassis_Init(ChassisData_Type *ChassisData);

/**
 * @brief 更新麦轮转速
 * @param XSpeed 左右 m/s
 * @param YSpeed 前后 m/s
 * @param WSpeed 旋转 rad/s 逆时针为正
 * @note 电机位置：左上角0,逆时针依次增加
 * @note 转子的角速度(rad/s) = 电机减速比 * 轮子角速度
 */
void Chassis_Update(ChassisData_Type *ChassisData, float XSpeed, float YSpeed, float WSpeed);

/**
 * @brief 修正旋转后底盘的前进方向
 * @param angle 期望的前进方向
 */
void Chassis_Fix(ChassisData_Type *ChassisData, float angle);

/**
 * @brief 麦轮解算, 更新转子转速
 */
void Chassis_Calculate_Rotor_Speed(ChassisData_Type *ChassisData);

/**
 * @brief 设置转子速度上限 (rad/s)
 * @param wheelSpeed
 * @param rotorSpeed
 */
void Chassis_Limit_Rotor_Speed(ChassisData_Type *ChassisData, float maxRotorSpeed);

/**
 * @brief 按比例更新速度
 * @param scale 速度缩放比例
 */
void Chassis_Scale_Rotor_Speed(ChassisData_Type *ChassisData, float scale);

/**
 * @brief 设置功率更新上限
 * @param maxPower       功率上限
 * @param targetPower    目标功率
 * @param referencePower 参考功率 (来源裁判系统或电流计)
 * @param interval       任务周期
 */
void Chassis_Limit_Power(ChassisData_Type *cd, float targetPower, float referencePower, float referencePowerBuffer, float interval);
#endif
