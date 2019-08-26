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
TIM_ICInitTypeDef       TIM3_ICInitStructure;
TIM_ICInitTypeDef       TIM9_ICInitStructure;
TIM_ICInitTypeDef       TIM4_ICInitStructure;
TIM_ICInitTypeDef       TIM8_ICInitStructure;

void BSP_Landing_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
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
    // Catapult
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

void BSP_TIM3CH3_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // 下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3); // PA0 复用位定时器 2

    TIM_TimeBaseInitStructure.TIM_Period        = 0xFFFF; // 设置ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 90 - 1; // 设置分频
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM3_ICInitStructure.TIM_Channel     = TIM_Channel_3;            // TIM2通道1
    TIM3_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;    // 上升沿捕获
    TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1
    TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM3_ICInitStructure.TIM_ICFilter    = 0x00; // 不滤波
    TIM_ICInit(TIM3, &TIM3_ICInitStructure);
    TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_CC3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void BSP_TIM4CH1_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // 下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); // PA0 复用位定时器 2

    TIM_TimeBaseInitStructure.TIM_Period        = 0xFFFF; // 设置ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 90 - 1; // 设置分频
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    TIM4_ICInitStructure.TIM_Channel     = TIM_Channel_1;            // TIM2通道1
    TIM4_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;    // 上升沿捕获
    TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1
    TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM4_ICInitStructure.TIM_ICFilter    = 0x00; // 不滤波
    TIM_ICInit(TIM4, &TIM4_ICInitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update | TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void BSP_TIM9CH1_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // 下拉
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9); // PA0 复用位定时器 2

    TIM_TimeBaseInitStructure.TIM_Period        = 0xFFFF;  // 设置ARR
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 180 - 1; // 设置分频
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseInitStructure);

    TIM9_ICInitStructure.TIM_Channel     = TIM_Channel_1;            // TIM2通道1
    TIM9_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;    // 上升沿捕获
    TIM9_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1
    TIM9_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM9_ICInitStructure.TIM_ICFilter    = 0x00; // 不滤波
    TIM_ICInit(TIM9, &TIM9_ICInitStructure);
    TIM_ITConfig(TIM9, TIM_IT_Update | TIM_IT_CC1, ENABLE);
    TIM_Cmd(TIM9, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_BRK_TIM9_IRQn;
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
}

void BSP_Limit_Switch(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void BSP_Supply_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 时钟使能

    // GPIO
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, TIM4);   // GPIO复用为定时器
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;       // GPIO
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;      // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;      // 上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);             // 初始化

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); // GPIO复用为定时器

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 9000 - 1;           // 定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period        = 200 - 1;            // 自动重装载值
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       // ClockDivision
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);               // 初始化定时器

    // TIM_OC
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;        // 选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_Low;     // 输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse       = 14;
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);          // 根据指定的参数初始化外设
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); // 使能TIM在CCR1上的预装载寄存器
    TIM_ARRPreloadConfig(TIM4, ENABLE);               // ARPE使能
    TIM_Cmd(TIM4, ENABLE);                            // 使能TIM
}

void BSP_Fsm_Counter_Init(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period        = 1000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 90 - 1;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = TIM8_TRG_COM_TIM14_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM14, ENABLE);
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
    BSP_Fsm_Counter_Init();

    // GPIO输出配置
    BSP_Landing_Init();         //  PH11  多加四个IO口读取光电开关
    BSP_Rescue_Init();          // PI0
    BSP_Take_Init();            // Take: PA3 Rotate: PA2 Catapult: PA1
    BSP_Optoelectronic_Input(); // 取弹: PI7 PI6 PI5 PI2
    BSP_Limit_Switch();         // 限位开关: L:PC2 R:PC3
    BSP_Supply_Init();          // 补给舵机初始化

    // 补给舵机输出
    // BSP_PWM_Set_Port(&PWM_Supply1, PWM_PORT_PD14);
    // BSP_PWM_Init(&PWM_Supply1, 9000, 200, TIM_OCPolarity_Low);
    // BSP_PWM_Set_Port(&PWM_Supply2, PWM_PORT_PD15);
    // BSP_PWM_Init(&PWM_Supply2, 9000, 200, TIM_OCPolarity_Low);

    // 图传舵机输出
    BSP_PWM_Set_Port(&PWM_Image_Yaw, PWM_PORT_PD13);
    BSP_PWM_Init(&PWM_Image_Yaw, 9000, 200, TIM_OCPolarity_Low);

    // 救援摄像头舵机输出
    BSP_PWM_Set_Port(&PWM_Visual_Rescue, PWM_PORT_PD12);
    BSP_PWM_Init(&PWM_Visual_Rescue, 9000, 200, TIM_OCPolarity_Low);

    // 板间通信
    // Board (UART7)
    BSP_UART7_Init(115200, USART_IT_IDLE);
    BSP_DMA_UART7_RX_Init(Board.receiveBuf, Protocol_Buffer_Length);
    BSP_DMA_UART7_TX_Init(Board.sendBuf, Protocol_Buffer_Length);

    // 输入捕获
    BSP_TIM5CH1_Init(); // PH10
    BSP_TIM2CH1_Init(); // PA0
    // BSP_TIM4CH1_Init(); // PD12
    BSP_TIM3CH3_Init(); // PB0
    BSP_TIM9CH1_Init(); // PE5
}
// 空余: PH12 PE4 PE5 PE6 PE12 PA2
