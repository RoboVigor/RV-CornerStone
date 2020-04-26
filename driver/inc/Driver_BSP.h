/**
 * @file Driver_BSP.h
 * @brief BSP初始化
 */

#ifndef __DRIVER_BSP_H
#define __DRIVER_BSP_H

#include "stm32f4xx.h"
#include "beep.h"

#define RCC_APB1 1
#define RCC_APB2 2

#define PWM_PD12 0x408210cc
#define PWM_PD13 0x408220cd
#define PWM_PD14 0x408230ce
#define PWM_PD15 0x408240cf
#define PWM_PH10 0x80c211ca
#define PWM_PH11 0x80c221cb
#define PWM_PH12 0x80c231cc
#define PWM_PI0 0x80c24200
#define PWM_PA0 0x10011000
#define PWM_PA1 0x10012001
#define PWM_PA2 0x10013002
#define PWM_PA3 0x10014003
#define PWM_PI5 0x21431205
#define PWM_PI6 0x21432206
#define PWM_PI7 0x21433207
#define PWM_PI2 0x21434202
#define PWM_PA8 0x11011008
#define PWM_PA9 0x11012009
#define PWM_PA10 0x1101300a
#define PWM_PA11 0x1101400b
#define PWM_PC1 0x11011081
#define PWM_PC2 0x11012082
#define PWM_PC3 0x11013083
#define PWM_PC4 0x11014084
#define PWM_PC5 0x21431085
#define PWM_PC6 0x21432086
#define PWM_PC7 0x21433087

typedef struct {
    uint32_t      RCC_APBxPeriph_TIMx;
    uint32_t      TIMx_BASE;
    uint8_t       GPIO_AF_TIMx;
    uint8_t       Channel;
    uint32_t      GPIOx_BASE;
    uint32_t      GPIO_PinSourcex;
    uint32_t      RCC_AHB1Periph_GPIOx;
    uint16_t      GPIO_Pin_x;
    uint8_t       CCRx;
    GPIO_TypeDef *GPIOx;
    TIM_TypeDef * TIMx;
} PWM_Type;

// SERVICE
void BSP_CAN_Init(void);
void BSP_DBUS_Init(uint8_t *remoteBuffer);
void BSP_IMU_Init(void);
void BSP_Laser_Init(void);
void BSP_User_Power_Init(void);

// NOT IMPLEMENTED
void BSP_I2C2_Init(void);
void BSP_Button_Init(void);

// TEMPORARY USE
void BSP_TIM2_Init(void);

// DMA
void BSP_DMA_USART3_RX_Init(uint32_t DMA_Memory0BaseAddr, uint32_t DMA_BufferSize);
void BSP_DMA_USART6_RX_Init(uint32_t DMA_Memory0BaseAddr, uint32_t DMA_BufferSize);

// USART
void BSP_USART_Init(
    USART_TypeDef *, uint32_t, uint8_t, uint8_t, uint8_t, uint16_t, GPIO_TypeDef *, uint16_t, uint32_t, uint16_t, uint16_t, uint16_t, uint32_t, uint16_t);
void BSP_USART2_Init(uint32_t baudRate, uint16_t interruptFlag);
void BSP_USART3_Init(uint32_t baudRate, uint16_t interruptFlag);
void BSP_USART6_Init(uint32_t baudRate, uint16_t interruptFlag);
void BSP_UART7_Init(uint32_t baudRate, uint16_t interruptFlag);
void BSP_UART8_Init(uint32_t baudRate, uint16_t interruptFlag);

// PWM
void BSP_PWM_Set_Port(PWM_Type *PWMx, uint32_t PWM_PORT_Px);
void BSP_PWM_Init(PWM_Type *PWMx, uint16_t prescaler, uint32_t period, uint16_t polarity);
void PWM_Set_Compare(PWM_Type *PWMx, uint32_t compare);

// LED
void BSP_LED_Init(void);
void LED_Set_Row(uint16_t row);
void LED_Set_Progress(uint16_t progress);
void LED_Run_Horse();
void LED_Run_Horse_XP();

// BEEP
void    BSP_Beep_Init(void);
uint8_t KTV_Play(Song_Type song);

#endif
