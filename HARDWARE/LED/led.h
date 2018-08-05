#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED锟剿口讹拷锟斤拷
#define LED1 PFout(14)	// DS0
#define LED2 PEout(7)	// DS1	 

#define GREEN_LIGHT_ON  	GPIO_ResetBits(GPIOF,GPIO_Pin_14)
#define GREEN_LIGHT_OFF 	GPIO_SetBits(GPIOF,GPIO_Pin_14)
#define GREEN_LIGHT_TOGGLE	GPIO_ToggleBits(GPIOF,GPIO_Pin_14)

#define RED_LIGHT_ON    	GPIO_ResetBits(GPIOE,GPIO_Pin_7)
#define RED_LIGHT_OFF     	GPIO_SetBits(GPIOE,GPIO_Pin_7)
#define RED_LIGHT_TOGGLE  	GPIO_ToggleBits(GPIOE,GPIO_Pin_7)


void LED_Init(void);//锟斤拷始锟斤拷	

#endif
