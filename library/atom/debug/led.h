#ifndef __LED_H
#define __LED_H
#include "sys.h"

#ifdef STM32F427_437xx
#define LED1 PFout(14) // DS0
#define LED2 PEout(7)  // DS1

#define GREEN_LIGHT_ON GPIO_ResetBits(GPIOF, GPIO_Pin_14)
#define GREEN_LIGHT_OFF GPIO_SetBits(GPIOF, GPIO_Pin_14)
#define GREEN_LIGHT_TOGGLE GPIO_ToggleBits(GPIOF, GPIO_Pin_14)

#define RED_LIGHT_ON GPIO_ResetBits(GPIOE, GPIO_Pin_11)
#define RED_LIGHT_OFF GPIO_SetBits(GPIOE, GPIO_Pin_11)
#define RED_LIGHT_TOGGLE GPIO_ToggleBits(GPIOE, GPIO_Pin_11)
#endif
#ifdef STM32F407xx
#define LED1 PHout(12) // DS0
#define LED2 PHout(11) // DS1
#define LED2 PHout(10) // DS2

#define BLUE_LIGHT_ON GPIO_SetBits(GPIOH, GPIO_Pin_10)
#define BLUE_LIGHT_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_10)
#define BLUE_LIGHT_TOGGLE GPIO_ToggleBits(GPIOH, GPIO_Pin_10)

#define GREEN_LIGHT_ON GPIO_SetBits(GPIOH, GPIO_Pin_11)
#define GREEN_LIGHT_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_11)
#define GREEN_LIGHT_TOGGLE GPIO_ToggleBits(GPIOH, GPIO_Pin_11)

#define RED_LIGHT_ON GPIO_SetBits(GPIOH, GPIO_Pin_12)
#define RED_LIGHT_OFF GPIO_ResetBits(GPIOH, GPIO_Pin_12)
#define RED_LIGHT_TOGGLE GPIO_ToggleBits(GPIOH, GPIO_Pin_12)
#endif

void LED_Init(void);

#endif
