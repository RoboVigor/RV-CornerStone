#include "key.h"
#include "delay.h"

//按键初始化函数
void KEY_Init(void) {

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //使能GPIOD时钟

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;       // KEY对应引脚
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;      //普通输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100M
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;      //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);             //初始化GPIOD10
}
