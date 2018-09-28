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
    int   output;
    int   lastOutput;

} PID_Type;

void PID_Init(PID_Type *pid, float p, float i, float d, float maxOutput, float maxOutput_I);
int  PID_Calculate(PID_Type *pid, float target, float feedback);
int  PID_Increment_Calculate(PID_Type *pid, float target, float feedback);
void PID_Print(PID_Type *pid);

#endif
