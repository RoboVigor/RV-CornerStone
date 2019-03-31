/**
 * @brief 用户BSP
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"

void BSP_USER_Init(void) {
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // 比上一行少个 Init hhhhh
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    TIM_BDTRInitTypeDef     TIM8_BDTRInitStruct;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    /**** PUBLIC ****/

    // VIEW MODE CHANGE - PI6 - TIM8 Channel 2 - PWM OUTPUT

    // RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

    // GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_TIM8);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_TIM8);

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 9000 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        = 200 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;        //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;     //输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse       = 5;
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);          //根据指定的参数初始化外设TIM8 4OC1
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); //使能TIM8在CCR1上的预装载寄存器
    TIM_ARRPreloadConfig(TIM8, ENABLE);               // ARPE使能
    TIM_Cmd(TIM8, ENABLE);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

    /**** Landing ****/

    // Switch - PH11
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);

    // Power - PH12
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);

    /**** RESCUE ****/

    // HOOK - PI0

    /**** RESCUE END ****/

    /**** SUPPLY ****/

    // GATE1 - PD15 - TIM4 Channel 4 - PWM OUTPUT

    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  // TIM4时钟使能
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //使能PORTD时钟

    // GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4); // GPIOD15复用为定时器4

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 9000 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        = 200 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_Pulse       = 5;
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    // GATE2 - PH10 - TIM5 Channel 1 - PWM OUTPUT

    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);  // TIM5时钟使能
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); //使能PORTD时钟

    // GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5); // GPIOH10复用为定时器5

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;             // GPIOH10
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;            //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;           //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;            //上拉
    GPIO_Init(GPIOH, &GPIO_InitStructure);                   //初始化
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5); // GPIOH10复用为定时器5

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 9000 - 1;           //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period        = 200 - 1;            //自动重装载值
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure); //初始化定时器5

    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;        //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;     //输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse       = 25;
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);          //根据指定的参数初始化外设TIM1 4OC1
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable); //使能TIM5在CCR1上的预装载寄存器
    TIM_ARRPreloadConfig(TIM5, ENABLE);               // ARPE使能
    TIM_Cmd(TIM5, ENABLE);                            //使能TIM5

    /**** TAKE ****/

    // Rotate - PA2
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Take - PA3
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Distance_Sensor - PH10 - TIM5 Channel 1 - PWM INPUT CAPTURE
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // 下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5); // PH10 复用位定时器 5

    // RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    // TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period        = 0XFFFFFFFF; // 设置ARR
    TIM_TimeBaseStructure.TIM_Prescaler     = 84 - 1;     // 设置分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_ICInitTypeDef TIM5_ICInitStructure;
    TIM5_ICInitStructure.TIM_Channel     = TIM_Channel_1;            // TIM5通道1
    TIM5_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;    // 上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM5_ICInitStructure.TIM_ICFilter    = 0x00; // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);
    TIM_ITConfig(TIM5, TIM_IT_Update | TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM5, ENABLE);

    // NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /**** TAKE END ****/

    // GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    // GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    // GPIO_Init(GPIOI, &GPIO_InitStructure);
}

/**** Distance_Sensor ****/
// // 调试传感器(USART)
// // 设置时钟
// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
// RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

// // 复用GPIO口
// GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
// GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

// GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
// GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
// GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
// GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// GPIO_Init(GPIOD, &GPIO_InitStructure);

// // USART 初始化
// USART_InitTypeDef USART_InitStructure;
// USART_InitStructure.USART_BaudRate            = 9600;
// USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
// USART_InitStructure.USART_StopBits            = USART_StopBits_1;
// USART_InitStructure.USART_Parity              = USART_Parity_No;
// USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
// USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
// USART_Init(USART2, &USART_InitStructure);

// // 使能 USART
// USART_Cmd(USART2, ENABLE);

// // USART中断配置
// NVIC_InitTypeDef NVIC_InitStructure;
// NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
// NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
// NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
// NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
// NVIC_Init(&NVIC_InitStructure);

// // 中断模式设置
// USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 每接收一个字节进入一次中断
/**** Distance_Sensor END ****/
