/**
 * @file Driver_Chassis.h
 * @brief 底盘运动驱动
 * @version 0.5
 * - Can_Send() 新增了电调标识符id参数,所有CAN发送只需要这一个函数就能够完成
 */

#ifndef __Driver_PID_H
#define __Driver_PID_H
typedef struct {
    // required
    float p;
    float i;
    float d;
    float maxOutput;

    // optional
    float maxOutput_I;

    // input
    float target;
    float feedback;
    float lastTarget;

    // calculation
    float error;
    float lastError;

    // output
    float output_P;
    float output_I;
    float output_D;
    float output;
    float lastOutput;

} PID_Type;

/**
 * @brief PID初始化
 *
 * @param pid PID结构体
 * @param p P系数
 * @param i I系数
 * @param d D系数
 * @param maxOutput 最大输出值
 * @param maxOutput_I 积分最大输出值
 */
void PID_Init(PID_Type *pid, float p, float i, float d, float maxOutput, float maxOutput_I);

/**
 * @brief 位置式PID计算
 *
 * @param pid PID结构句柄
 * @param target 目标量
 * @param feedback 反馈量
 * @return float
 */
float PID_Calculate(PID_Type *pid, float target, float feedback);

/**
 * @brief 增量式PID计算
 * @note 该模式下maxOutput和maxOutput_I无效
 *
 * @param pid PID结构句柄
 * @param target 目标量
 * @param feedback 反馈量
 * @return float
 */
float PID_Increment_Calculate(PID_Type *pid, float target, float feedback);

/**
 * @brief 打印PID结构体
 */
void PID_Print(PID_Type *pid);

#endif
