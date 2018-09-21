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

    // output0
    float output_P;
    float output_I;
    float output_D;
    int   output;
    int   lastOutput;

    // int Iindex;   //删除
    // int PIDindex; //删除
    // int Portion;  //删除

} PID_Type;

void PID_Init(PID_Type *pid, float p, float i, float d, float maxOutput, float maxOutput_I);
int  PID_Calculate(PID_Type *pid, float target, float feedback);
int  Increment_PID_Calculate(PID_Type *pid, float target, float feedback);

#endif
