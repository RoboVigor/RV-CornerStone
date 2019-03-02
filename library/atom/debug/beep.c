#include "beep.h"

//初始化PB4为输出口
//BEEP IO初始化
void Beep_Init(void)
{
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  	//TIM3时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3); //GPIOB4复用为定时器3

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;           //GPIOB4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);              	//初始化PB4

	TIM_TimeBaseStructure.TIM_Prescaler = 90-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = 1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);//初始化定时器3

	//初始化TIM3 Channel1 PWM模式
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC1

	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);  //使能TIM3在CCR1上的预装载寄存器

  TIM_ARRPreloadConfig(TIM3,ENABLE);//ARPE使能

	TIM_Cmd(TIM3, ENABLE);  //使能TIM3

}

const uint16_t tone_tab[] =
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};

const Sound_Tone_Type Mavic_Startup_music[Startup_Success_music_len] =
{
  So5L, So5L, So5L, So5L, La6L, La6L, La6L, La6L, Mi3M, Mi3M, Mi3M, Mi3M, Mi3M, Silent,
};

//
void Sing(Sound_Tone_Type tone)
{
  if(Silent == tone)
    BEEP_CH = 0;
  else
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}

//play the start up music
void Sing_Startup_music(uint32_t index)
{
  if(index < Startup_Success_music_len)
    Sing(Mavic_Startup_music[index]);
}




