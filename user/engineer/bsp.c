/**
 * @brief 用户BSP
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "handle.h"
#include "Driver_BSP.h"

GPIO_InitTypeDef        GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
TIM_OCInitTypeDef       TIM_OCInitStructure;
NVIC_InitTypeDef        NVIC_InitStructure;
TIM_ICInitTypeDef       TIM2_ICInitStructure;
TIM_ICInitTypeDef       TIM5_ICInitStructure;

void BSP_Landing_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
    // Front
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);
    // Behind
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
    // Power
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);
}

void BSP_Rescue_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
}

void BSP_Take_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // Take 
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Rotate
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void BSP_Pushrod_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    // Common
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    // Power
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void BSP_TIM5CH1_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOH, &GPIO_InitStructure);                   //初始化
    GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5); // GPIOH10复用为定时器5

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 90 - 1;             //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period        = 0xFFFFFFFF;         //自动重装载值
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure); //初始化定时器5

    TIM5_ICInitStructure.TIM_Channel     = TIM_Channel_1;            // TIM5通道1
    TIM5_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;    // 上升沿捕获
    TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1
    TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM5_ICInitStructure.TIM_ICFilter    = 0x00; // 不滤波
    TIM_ICInit(TIM5, &TIM5_ICInitStructure);
    TIM_ITConfig(TIM5, TIM_IT_Update | TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM5, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void BSP_TIM2CH1_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // 下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2); // PA0 复用位定时器 2

    TIM_TimeBaseInitStructure.TIM_Period        = 0xFFFFFFFF; // 设置ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 90 - 1;     // 设置分频
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM2_ICInitStructure.TIM_Channel     = TIM_Channel_1;            // TIM2通道1
    TIM2_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;    // 上升沿捕获
    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1
    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM2_ICInitStructure.TIM_ICFilter    = 0x00; // 不滤波
    TIM_ICInit(TIM2, &TIM2_ICInitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void BSP_Optoelectronic_Input(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    // 取弹对位
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOI, &GPIO_InitStructure);
    // 登岛对位
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_5 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void BSP_Limit_Switch(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void BSP_Init(void) {
    BSP_CAN_Init();
    BSP_DBUS_Init(remoteBuffer);
    // BSP_TIM2_Init();
    BSP_IMU_Init();
    BSP_USART3_Init(9600, 0);
    BSP_USART6_Init(9600, 0);
    BSP_UART7_Init(9600, 0);
    BSP_UART8_Init(9600, 0);
    BSP_Laser_Init();
    BSP_Beep_Init();
    BSP_LED_Init();
    BSP_User_Power_Init();

    // GPIO输出配置
    BSP_Landing_Init(); // front: PH11 behind: PI2 power: PH12  多加四个IO口读取光电开关
    BSP_Rescue_Init();  // PI0                                  
    BSP_Take_Init();    // Take: PA3 Rotate: PA2
    BSP_Pushrod_Init(); // power: PA1 common: PH12 PH11
    BSP_Optoelectronic_Input();   // 取弹: PI7 PI6 PI5 PI2 登岛: PE4 PE5 PE6 PE12
    BSP_Limit_Switch(); // 限位开关: PC2 PC3 

    // 补给舵机输出 
    BSP_PWM_Set_Port(&PWM_Supply1, PWM_PORT_PD14);
    BSP_PWM_Init(&PWM_Supply1, 9000, 200, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Supply2, PWM_PORT_PD15);
    BSP_PWM_Init(&PWM_Supply2, 9000, 200, TIM_OCPolarity_Low);

    // 图传舵机输出
    BSP_PWM_Set_Port(&PWM_Image_Yaw, PWM_PORT_PD13);
    BSP_PWM_Init(&PWM_Image_Yaw, 9000, 200, TIM_OCPolarity_Low);

    // 救援摄像头舵机输出
    BSP_PWM_Set_Port(&PWM_Rescue, PWM_PORT_PD12);
    BSP_PWM_Init(&PWM_Rescue, 9000, 200, TIM_OCPolarity_Low);

    // 输入捕获
    BSP_TIM5CH1_Init(); // PH10
    BSP_TIM2CH1_Init(); // PA0
}