/**
 * @brief 用户BSP
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"

void BSP_USER_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);

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

    /**** Landing ****/
    // Switch
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);

    // Power
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);

    // GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6;
    // GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    // GPIO_Init(GPIOI, &GPIO_InitStructure);

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

    // TIM5输入捕获PWM波初始化
    // 测距传感器
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;  // 下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5); //PH10 复用位定时器 5 

    // 测距传感器时钟初始化
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
    TIM_TimeBaseStructure.TIM_Period        = 0XFFFFFFFF; // 设置ARR
    TIM_TimeBaseStructure.TIM_Prescaler     = 84-1;   // 设置分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
    
    TIM_ICInitTypeDef  TIM5_ICInitStructure; 
    TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; // TIM5通道1
    TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; // 上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
    TIM5_ICInitStructure.TIM_ICFilter = 0x00; // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);

    TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC1,ENABLE);
    TIM_Cmd(TIM5, ENABLE);

    // 中断初始化
    NVIC_InitTypeDef NVIC_InitStructure; 
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);

}
