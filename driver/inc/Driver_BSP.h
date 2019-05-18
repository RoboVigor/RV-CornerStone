/**
 * @file Driver_BSP.h
 * @brief BSP通用驱动
 */

#ifndef __DRIVER_BSP_H
#define __DRIVER_BSP_H

#include "stm32f4xx.h"

#define PWM_PORT_PD12 0x4021008c
#define PWM_PORT_PD13 0x4022008d
#define PWM_PORT_PD14 0x4023008e
#define PWM_PORT_PD15 0x4024008f
#define PWM_PORT_PH10 0x8021080a
#define PWM_PORT_PH11 0x8022080b
#define PWM_PORT_PH12 0x8023080c
#define PWM_PORT_PI0 0x80241000
#define PWM_PORT_PA0 0x10110010
#define PWM_PORT_PA1 0x10120011
#define PWM_PORT_PA2 0x10130012
#define PWM_PORT_PA3 0x10140013
#define PWM_PORT_PI5 0x21311005
#define PWM_PORT_PI6 0x21321006
#define PWM_PORT_PI7 0x21331007
#define PWM_PORT_PI2 0x21341002

typedef struct {
    uint32_t      RCC_APBxPeriph_TIMx;
    TIM_TypeDef * TIMx;
    uint8_t       GPIO_AF_TIMx;
    uint32_t      RCC_AHB1Periph_GPIOx;
    GPIO_TypeDef *GPIOx;
    uint32_t      GPIO_PinSourcex;
    uint16_t      GPIO_Pin_x;
    uint8_t       CCRx;
    uint8_t       Channel;
} PWM_Type;

void BSP_CAN_Init(void);
void BSP_DBUS_Init(uint8_t *remoteBuffer);
void BSP_IMU_Init(void);
void BSP_TIM2_Init(void);
void BSP_USART3_Init(uint32_t baudRate);
void BSP_USART6_Init(uint32_t baudRate);
void BSP_UART7_Init(uint32_t baudRate);
void BSP_UART8_Init(uint32_t baudRate);
void BSP_Laser_Init(void);
void BSP_User_Power_Init(void);
void BSP_PWM_Set_Port(PWM_Type *PWMx, uint32_t PWM_PORT_Px);
void BSP_PWM_Init(PWM_Type *PWMx, uint16_t prescaler, uint32_t period, uint16_t polarity);
void PWM_Set_Compare(PWM_Type *PWMx, uint32_t compare);
void BSP_DMA2_Init(void);
void BSP_I2C2_Init(void);
void BSP_LED_Init(void);
void BSP_Button_Init(void);
void BSP_Beep_Init(void);
#endif
