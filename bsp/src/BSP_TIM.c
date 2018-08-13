
#define __BSP_TIM_Global
#include "main.h"

/**
 * @brief  TIM初始化
 * @param  void
 * @return void
 */
void BSP_TIM_InitConfig(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TIM 频率
    GPIO_InitTypeDef        GPIO_InitStructure;        // TIM4_PWM GPIO口设置
    TIM_OCInitTypeDef       TIM_OCInitStructure;       // TIM4_PWM PWM模式

    // TIM2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitStructure.TIM_Period        = 2 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 9000 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM2, ENABLE);

    // TIM6
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 90 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_Period        = 1000 - 1;
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM6, ENABLE);

    // TIM4_PWM
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  // TIM4时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //使能PORTD时钟

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); // GPIOD12复用为定时器4

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;       // GPIOD12
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;      //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;      //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);             //初始化

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 9000 - 1;           //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period        = 200 - 1;            //自动重装载值
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure); //初始化定时器4

    //初始化TIM4 Channel1 PWM模式
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;        //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;     //输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse       = 5;
    TIM_OC1Init(TIM4, &TIM_OCInitStructure); //根据指定的参数初始化外设TIM1 4OC1

    TIM_OC1PreloadConfig(TIM4,
                         TIM_OCPreload_Enable); //使能TIM12在CCR1上的预装载寄存器

    TIM_ARRPreloadConfig(TIM4, ENABLE); // ARPE使能

    TIM_Cmd(TIM4, ENABLE); //使能TIM4
}
