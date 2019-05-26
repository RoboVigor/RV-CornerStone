#include "beep.h"

//初始化PB4为输出口
// BEEP IO初始化
void Beep_Init(void) {
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  // TIM3时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能GPIOB时钟

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3); // GPIOB4复用为定时器3

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;        // GPIOB4
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;      //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;      //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);             //初始化PB4

    TIM_TimeBaseStructure.TIM_Prescaler     = 90 - 1;             //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_Period        = 1;                  //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //初始化定时器3

    //初始化TIM3 Channel1 PWM模式
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;        //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;    //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);                      //根据T指定的参数初始化外设TIM3 OC1

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); //使能TIM3在CCR1上的预装载寄存器

    TIM_ARRPreloadConfig(TIM3, ENABLE); // ARPE使能

    TIM_Cmd(TIM3, ENABLE); //使能TIM3
}

const uint16_t tone_tab[] = {
    3822, 3405, 3033, 2863, 2551, 2272, 2024, // bass 1~7
    1911, 1702, 1526, 1431, 1275, 1136, 1012, // mid 1~7
    955,  851,  758,  715,  637,  568,  506,  // treble 1~7
};

// const Sound_Tone_Type Mavic_Startup_music[Startup_Success_music_len] =
// {
//   So5L, So5L, So5L, So5L, La6L, La6L, La6L, La6L, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Silent,
// };

const Sound_Tone_Type Mavic_Startup_music_earth[Startup_Success_music_len_earth] = {
    Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Silent, Mi3M,   Mi3M,   So5M,   So5M, La6M, La6M,   Silent,
    Silent, Silent, Silent, So5M,   So5M,   La6M,   La6M,   Silent, Silent, Silent, Silent, So5M,   So5M,   La6M,   La6M,   Do1H, Do1H, So5M,   So5M,
    La6M,   La6M,   Mi3M,   Mi3M,   Silent, Silent, Mi3M,   Mi3M,   So5M,   So5M,   La6M,   La6M,   Mi3H,   Mi3H,   Do1H,   Do1H, Re2H, Re2H,   La6M,
    La6M,   Silent, Silent, Mi3M,   Mi3M,   So5M,   So5M,   La6M,   La6M,   Silent, Silent, Silent, Silent, So5M,   So5M,   La6M, La6M, Silent, Silent,
    Silent, Silent, So5M,   So5M,   La6M,   La6M,   Do1H,   Do1H,   So5M,   So5M,   La6M,   La6M,   Mi3M,   Mi3M,   So5M,   So5M, Do1M, Do1M,   Re2M,
    Re2M,   Mi3M,   Mi3M,   Mi3M,   Mi3M,   Do1H,   Do1H,   Do1H,   Do1H,   La6M,   La6M,   La6M,   La6M,   Mi3H,   Mi3H,   Mi3H, Mi3H, Re2H,   Re2H,
    Mi3H,   Re2H,   Do1H,   Do1H,   Re2H,   Re2H,   La6M,   La6M,   La6M,   La6M,   Silent, Silent, Silent, Silent, La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   Do1H,   Re2H,   Mi3M,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   So5M,   So5M,   Silent, So5M,   So5M,   La6M,   La6M,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   Do1H,   Re2H,   Mi3M,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   Fa4H,   Fa4H,   Silent, Fa4H,   Fa4H,   Mi3H,   Mi3H,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   Do1H,   Re2H,   Mi3M,   La6M,   La6M, La6M, La6M,   Silent,
    La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   So5M,   So5M,   Silent, So5M,   So5M,   La6M,   La6M,   Silent, La6M, La6M, La6M,   La6M,
    Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M,   Silent, La6M,   Do1H,   Re2H,   Mi3M,   La6H, La6H, La6H,   La6H,
    La6H,   La6H,   So5M,   So5M,   Silent, So5M,   So5M,   La6M,   La6M,   Silent, La6M,   La6M,   La6M,   La6M};

const Sound_Tone_Type Mavic_Startup_music_sky[Startup_Success_music_len_sky] = {
    La6M, Si7M,   Do1H,   Do1H, Do1H,   Si7M,   Do1H,   Do1H,   Mi3H, Mi3H,   Si7M,   Si7M,   Si7M,  Si7M, Silent, Mi3M,   Mi3M,   La6M,   La6M,
    La6M, So5M,   La6M,   La6M, Do1H,   Do1H,   So5M,   So5M,   So5M, So5M,   Silent, Mi3M,   Mi3M,  Fa4M, Fa4M,   Fa4M,   Mi3M,   Fa4M,   Fa4M,
    Do1H, Do1H,   Mi3M,   Mi3M, Mi3M,   Mi3M,   Silent, Silent, Do1H, Do1H,   Do1H,   Si7M,   Si7M,  Fa4M, Fa4M,   Fa4M,   Si7M,   Si7M,   Silent,
    Si7M, Si7M,   Si7M,   Si7M, Silent, Silent, Silent, La6M,   Si7M, Do1H,   Do1H,   Do1H,   Si7M,  Do1H, Do1H,   Mi3H,   Mi3H,   Si7M,   Si7M,
    Si7M, Si7M,   Silent, Mi3M, Mi3M,   La6M,   La6M,   La6M,   So5M, La6M,   La6M,   Do1H,   Do1H,  So5M, So5M,   So5M,   So5M,   Silent, Re2M,
    Mi3M, Fa4M,   Fa4M,   Do1H, Si7M,   Si7M,   Do1H,   Do1H,   Re2H, Re2H,   Mi3H,   Do1H,   Do1H,  Do1H, Do1H,   Silent, Do1H,   Si7M,   La6M,
    La6M, Si7M,   Si7M,   So5M, So5M,   La6M,   La6M,   La6M,   La6M, Silent, Do1H,   Re2H,   Mi3H,  Mi3H, Mi3H,   Re2H,   Mi3H,   Mi3H,   So5H,
    So5H, Re2H,   Re2H,   Re2H, Re2H,   Silent, So5M,   So5M,   Do1H, Do1H,   Do1H,   Si7M,   Do1H,  Do1H, Mi3H,   Mi3H,   Mi3H,   Mi3H,   Mi3H,
    Mi3H, Silent, La6M,   Si7M, Do1H,   Do1H,   Si7M,   Si7M,   Re2H, Re2H,   Do1H,   Do1H,   Do1H,  So5M, So5M,   So5M,   So5M,   Silent, Fa4H,
    Fa4H, Mi3H,   Mi3H,   Re2H, Re2H,   Do1H,   Do1H,   Mi3H,   Mi3H, Mi3H,   Mi3H,   Silent, Mi3H,  Mi3H, La6H,   La6H,   La6H,   So5H,   So5H,
    So5H, Mi3H,   Re2H,   Do1H, Do1H,   Do1H,   Silent, Silent, Do1H, Re2H,   Re2H,   Do1H,   Re2H,  Re2H, Re2H,   So5H,   So5H,   Mi3H,   Mi3H,
    Mi3H, Mi3H,   Silent, Mi3H, Mi3H,   La6H,   La6H,   La6H,   So5H, So5H,   So5H,   Mi3H,   Re2H,  Do1H, Do1H,   Do1H,   Silent, Silent, Do1H,
    Re2H, Re2H,   Do1H,   Re2H, Re2H,   Re2H,   Si7M,   Si7M,   La6M, La6M,   La6M,   La6M,   Silent};

const Sound_Tone_Type Mavic_Startup_music_soul[Startup_Success_music_len_soul] = {
    Mi3M, Mi3M, Mi3M, Mi3M, So5M,   So5M, So5M, So5M,   La6M, La6M, La6M, La6M, La6M, La6M,   La6M, La6M, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M,   Mi3M,
    Re2M, Re2M, Re2M, Re2M, Do1M,   Do1M, Do1M, Do1M,   Re2M, Re2M, Re2M, Re2M, Mi3M, Mi3M,   Mi3M, Mi3M, So5M, So5M, So5M, So5M, So5M, So5M, So5M,   So5M,
    Mi3M, Mi3M, Mi3M, Mi3M, Mi3M,   Mi3M, Mi3M, Mi3M,   Re2M, Re2M, Re2M, Re2M, Do1M, Do1M,   Do1M, Do1M, Re2M, Re2M, Re2M, Re2M, Mi3M, Mi3M, Mi3M,   Mi3M,
    Do1M, Do1M, Do1M, Do1M, Silent, Do1M, Do1M, Do1M,   Do1M, Mi3M, Mi3M, Mi3M, Mi3M, Silent, Mi3M, Mi3M, Mi3M, Mi3M, Do1M, Do1M, Do1M, Do1M, Silent, Do1M,
    Do1M, Do1M, Do1M, Mi3M, Mi3M,   Mi3M, Mi3M, Silent, Mi3M, Mi3M, Mi3M, Mi3M, Do1M, Do1M,   Do1M, Do1M, Do1M, Do1M, Do1M, Do1M};

void Sing(Sound_Tone_Type tone) {
    if (Silent == tone)
        BEEP_CH = 0;
    else {
        BEEP_ARR = tone_tab[tone];
        BEEP_CH  = tone_tab[tone] / 2;
    }
}

// play the start up music
void Sing_Startup_music(Song_Type Song, uint32_t index) {
    if (Song == Sky)
        if (index < Startup_Success_music_len_sky) Sing(Mavic_Startup_music_sky[index]);
    if (Song == Earth)
        if (index < Startup_Success_music_len_earth) Sing(Mavic_Startup_music_earth[index]);
    if (Song == Soul)
        if (index < Startup_Success_music_len_soul) Sing(Mavic_Startup_music_soul[index]);
}
