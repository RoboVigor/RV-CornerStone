/**
 * @file Driver_BSP.h
 * @brief BSP通用驱动
 */

#ifndef __DRIVER_BSP_H
#define __DRIVER_BSP_H

#include "stm32f4xx.h"

#define PWM_PD12 0x4028008c
#define PWM_PD13 0x4028008d
#define PWM_PD14 0x4028008e
#define PWM_PD15 0x4028008f
#define PWM_PH10 0x802c080a
#define PWM_PH11 0x802c080b
#define PWM_PH12 0x802c080c
#define PWM_PI0 0x802c1000
#define PWM_PA0 0x10100010
#define PWM_PA1 0x10100011
#define PWM_PA2 0x10100012
#define PWM_PA3 0x10100013
#define PWM_PI5 0x21341005
#define PWM_PI6 0x21341006
#define PWM_PI7 0x21341007
#define PWM_PI2 0x21341002

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
void BSP_PWM_Init(uint32_t PWM_Px， uint16_t prescaler, uint32_t period);
void BSP_DMA2_Init(void);
void BSP_I2C2_Init(void);
void BSP_LED_Init(void);
void BSP_Button_Init(void);
void BSP_Beep_Init(void);
#endif
