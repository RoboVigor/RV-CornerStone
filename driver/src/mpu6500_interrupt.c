#include "config.h"
#include "mpu6500_i2c.h"
#include "mpu6500_interrupt.h"
#include "MPU6500_IST8310.h"
#include "mpu6500_driver.h"

#if BOARD_VERSION == 0
void MPU6500_IntConfiguration(void) {
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;
    EXTI_InitTypeDef exti;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    gpio.GPIO_Pin   = GPIO_Pin_1;
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &gpio);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, GPIO_PinSource1);
    exti.EXTI_Line    = EXTI_Line1;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel                   = EXTI1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

void EXTI1_IRQHandler(void) //中断频率1KHz
{
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
        EXTI_ClearFlag(EXTI_Line1);
        EXTI_ClearITPendingBit(EXTI_Line1);
        MPU6500_getMotion6();
    }
}

#endif

#if BOARD_VERSION == 1

void MPU6500_IntConfiguration(void) {
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;
    EXTI_InitTypeDef exti;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin   = GPIO_Pin_8;
    gpio.GPIO_Mode  = GPIO_Mode_IN;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource8);
    exti.EXTI_Line    = EXTI_Line8;
    exti.EXTI_Mode    = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel                   = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 8;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);
}

void EXTI9_5_IRQHandler(void) //中断频率1KHz
{
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearFlag(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line8);
        MPU6500_getMotion6();
    }
}

#endif