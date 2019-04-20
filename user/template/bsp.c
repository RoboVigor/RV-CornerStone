/**
 * @brief 用户BSP
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"

void BSP_USER_Init(void) {
    // 多通道输出比较
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    // TIM4
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    GPIO_Init(GPIOD,&GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Prescaler=9000-1; 
    TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    TIM_TimeBaseInitStructure.TIM_Period=200-1;   //自动重装载值 
    TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;  
    TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure); 

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
    TIM_OCInitStructure.TIM_Pulse = 5;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 
    TIM_ARRPreloadConfig(TIM4,ENABLE);

    TIM_Cmd(TIM4, ENABLE); 

    // // TIM5
    // GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
    // GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
    // GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5);
    // GPIO_PinAFConfig(GPIOI,GPIO_PinSource0,GPIO_AF_TIM5);

    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12; 
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    // GPIO_Init(GPIOH,&GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    // GPIO_Init(GPIOI,&GPIO_InitStructure);

    // TIM_TimeBaseInitStructure.TIM_Prescaler=9000-1; 
    // TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    // TIM_TimeBaseInitStructure.TIM_Period=200-1;   //自动重装载值 
    // TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;  
    // TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure); 

    // TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
    // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
    // TIM_OCInitStructure.TIM_Pulse = 5;
    // TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    // TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); 
    // TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    // TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable); 
    // TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    // TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable); 
    // TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    // TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); 
    // TIM_ARRPreloadConfig(TIM5,ENABLE);

    // TIM_Cmd(TIM5, ENABLE); 

    // // TIM2
    // GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
    // GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
    // GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
    // GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);

    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3; 
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    // GPIO_Init(GPIOA,&GPIO_InitStructure);

    // TIM_TimeBaseInitStructure.TIM_Prescaler=9000-1; 
    // TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    // TIM_TimeBaseInitStructure.TIM_Period=200-1;   //自动重装载值 
    // TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;  
    // TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure); 

    // TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
    // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
    // TIM_OCInitStructure.TIM_Pulse = 5;
    // TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    // TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); 
    // TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    // TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable); 
    // TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    // TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); 
    // TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    // TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); 
    // TIM_ARRPreloadConfig(TIM2,ENABLE);

    // TIM_Cmd(TIM2, ENABLE); 

    // TIM8
    // GPIO_PinAFConfig(GPIOI,GPIO_PinSource5,GPIO_AF_TIM8);
    // GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8);
    // GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8);
    // GPIO_PinAFConfig(GPIOI,GPIO_PinSource2,GPIO_AF_TIM8);

    // GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_2; 
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
    // GPIO_Init(GPIOI,&GPIO_InitStructure);

    // TIM_TimeBaseInitStructure.TIM_Prescaler=9000-1; 
    // TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    // TIM_TimeBaseInitStructure.TIM_Period=200-1;   //自动重装载值 
    // TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;  
    // TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure); 

    // TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 
    // TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    // TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 
    // TIM_OCInitStructure.TIM_Pulse = 5;
    // TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    // TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); 
    // TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    // TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); 
    // TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    // TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable); 
    // TIM_OC4Init(TIM8, &TIM_OCInitStructure);
    // TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); 
    // TIM_ARRPreloadConfig(TIM8,ENABLE);

    // TIM_Cmd(TIM8, ENABLE); 

    // TIM_CtrlPWMOutputs(TIM8, ENABLE);

//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//   GPIO_Init(GPIOI, &GPIO_InitStructure);
//   GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_TIM8);

//   TIM_TimeBaseInitStructure.TIM_Prescaler = 9000 - 1;
//   TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
//   TIM_TimeBaseInitStructure.TIM_Period = 200 - 1;
//   TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//   TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);

//   TIM_OCInitStructure.TIM_OCMode =
//       TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2
//   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
//   TIM_OCInitStructure.TIM_OCPolarity =
//       TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
//   TIM_OCInitStructure.TIM_Pulse = 5;
//   TIM_OC2Init(TIM8, &TIM_OCInitStructure); //根据指定的参数初始化外设TIM8 4OC1
//   TIM_OC2PreloadConfig(TIM8,
//                        TIM_OCPreload_Enable); //使能TIM8在CCR1上的预装载寄存器
//   TIM_ARRPreloadConfig(TIM8, ENABLE); // ARPE使能
//   TIM_Cmd(TIM8, ENABLE);
//   TIM_CtrlPWMOutputs(TIM8, ENABLE);
}
