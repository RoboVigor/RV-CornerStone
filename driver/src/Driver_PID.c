#include "Driver_PID.h"
#include "macro.h"
#include "handle.h"

/**
 * @brief PID初始化
 *
 * @param pid PID结构句柄
 * @param p P系数
 * @param i I系数
 * @param d D系数
 * @param maxOutput 最大输出值
 */
void PID_Init(PID_Type *pid, float p, float i, float d, float maxOutput, float maxOutput_I) {
    pid->p         = p;
    pid->i         = i;
    pid->d         = d;
    pid->maxOutput = maxOutput;

    pid->maxOutput_I = maxOutput_I;
    pid->target      = 0;
    pid->lastTarget  = 0;
    pid->feedback    = 0;
    pid->error       = 0;
    pid->lastError   = 0;
    pid->output_P    = 0;
    pid->output_I    = 0;
    pid->output_D    = 0;
    pid->output      = 0;
    pid->lastOutput  = 0;
}

/**
 * @brief 计算PID输出值
 *
 * @param pid PID结构句柄
 * @param target 目标量
 * @param feedback 反馈量
 * @return int
 */
int PID_Calculate(PID_Type *pid, float target, float feedback) {
    pid->lastTarget = pid->target;
    pid->target     = target;
    pid->feedback   = feedback;
    pid->lastOutput = pid->output;

    pid->error = pid->target - pid->feedback;

    pid->output_P = pid->p * pid->error;

    pid->output_I += pid->i * pid->error;
    MIAO(pid->output_I, -(pid->maxOutput_I), pid->maxOutput_I);

    pid->output = (pid->output_P + pid->output_I + pid->output_D);
    MIAO(pid->output, -(pid->maxOutput), pid->maxOutput);

    pid->lastError = pid->error;

    return pid->output;
}

int PID_Increment_Calculate(PID_Type *pid, float target, float feedback) {
    pid->target   = target;
    pid->feedback = feedback;

    pid->lastOutput = pid->output;

    pid->error = pid->target - pid->feedback;

    pid->output += (pid->p + pid->i) * pid->error - pid->p * pid->lastError;

    pid->lastError = pid->error;

    return pid->output;
}

/**
 * @brief 输出PID参量
 *
 * @param pid
 */
void PID_Print(PID_Type *pid) {
    printf("PID(%f, %f, %d)\r\n", pid->target, pid->feedback, pid->output);
}
