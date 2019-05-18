#include "driver_engineer.h"
#include "handle.h"

// 打开补给舱门
void Turn_Supply(int State) {
    if (State != 0) {
        TIM_SetCompare3(TIM4, 100);
        TIM_SetCompare4(TIM4, 100);
    } else {
        TIM_SetCompare3(TIM4, 50);
        TIM_SetCompare4(TIM4, 50);
    }
}

// 等摄像头
// void Turn_Camera_Screen(int State) {
//     if (State != 0) {
//         TIM_SetCompare3(TIM3, 100);
//         TIM_SetCompare4(TIM3, 100);
//     } else {
//         TIM_SetCompare3(TIM3, 50);
//         TIM_SetCompare4(TIM3, 50);
//     }
// }

// void Turn_Camera_Rescue(int State) {
//     if (State != 0) {
//         TIM_SetCompare2(TIM8, 100);
//     } else {
//         TIM_SetCompare2(TIM8, 50);
//     }
// }

// 等测试机构
// void Turn_Motor_Upthrow_On(int State) {
//     PID_Init(&PID_Upthrow_Speed, 0, 0, 0, 4000, 2000);
//     PID_Init(&PID_Upthrow_Angle, 0, 0, 0, 4000, 2000);
//     if (State != 0) {
//         float rpm2rps     = 3.14 / 60;
//         int   targetAngle = 0;
//         int   lastSpeed   = 0;
//         int   lastAngle   = 0;
//         lastSpeed         = Motor_Upthrow.speed;
//         lastAngle         = Motor_Upthrow.angle;
//         PID_Calculate(&PID_Upthrow_Angle, targetAngle, lastAngle);
//         PID_Calculate(&PID_Upthrow_Speed, PID_Transmission_Angle.output, lastSpeed * rpm2rps);
//         Can_Send(CAN1, 0x1FF, 0, 0, 0, PID_Upthrow_Speed.output);
//     }
// }
