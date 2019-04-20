#include "driver_engineer.h"

void Turn_Magazine(int State) {
    if (State != 0) {
        TIM_SetCompare3(TIM4,100);
        TIM_SetCompare4(TIM4,100);
    } else {
        TIM_SetCompare3(TIM4,50);
        TIM_SetCompare4(TIM4,50);
    }
}

void Turn_Camera_Screen(int State) {
    if (State != 0) {
        TIM_SetCompare3(TIM3,100);
        TIM_SetCompare4(TIM3,100);
    } else {
        TIM_SetCompare3(TIM3,50);
        TIM_SetCompare4(TIM3,50);
    }
}

void Turn_Camera_Rescue(int State) {
    if (State != 0) {
        TIM_SetCompare2(TIM8,100);
    } else {
        TIM_SetCompare2(TIM8,50);
    }

}
