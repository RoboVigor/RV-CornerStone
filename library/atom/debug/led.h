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

void LED_Init(void);

#endif
