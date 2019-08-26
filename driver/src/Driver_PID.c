#include "Driver_PID.h"
#include "macro.h"

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

float PID_Calculate(PID_Type *pid, float target, float feedback) {
    pid->lastTarget = pid->target;
    pid->target     = target;
    pid->feedback   = feedback;
    pid->lastOutput = pid->output;

    pid->error = pid->target - pid->feedback;

    pid->output_P = pid->p * pid->error;

    pid->output_I += pid->i * pid->error;
    MIAO(pid->output_I, -(pid->maxOutput_I), pid->maxOutput_I);

    pid->output_D = pid->d * (pid->error - pid->lastError);

    pid->output = (pid->output_P + pid->output_I + pid->output_D);

    MIAO(pid->output, -(pid->maxOutput), pid->maxOutput);

    pid->lastError = pid->error;

    return pid->output;
}

float PID_Increment_Calculate(PID_Type *pid, float target, float feedback) {
    pid->target   = target;
    pid->feedback = feedback;

    pid->lastOutput = pid->output;

    pid->error = pid->target - pid->feedback;

    pid->output += (pid->p + pid->i) * pid->error - pid->p * pid->lastError;

    pid->lastError = pid->error;

    return pid->output;
}

void PID_Print(PID_Type *pid) {
    printf("PID(%f, %f, %d)\r\n", pid->target, pid->feedback, pid->output);
}
