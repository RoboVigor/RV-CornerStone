/**
 * @file Driver_BSP.h
 * @brief BSP通用驱动
 */

#ifndef __DRIVER_BSP_H
#define __DRIVER_BSP_H

#include "stm32f4xx.h"

void BSP_CAN_Init(void);
void BSP_DBUS_Init(uint8_t remoteBuffer);
void BSP_IMU_Init(void);
void BSP_TIM2_Init(void);
void BSP_USART3_Init(uint32_t baudRate);
void BSP_USART6_Init(uint32_t baudRate);
void BSP_UART7_Init(uint32_t baudRate);
void BSP_UART8_Init(uint32_t baudRate);
void BSP_Laser_Init(void);
void BSP_User_Power_Init(void);
void BSP_PWM_Init(uint32_t channel);
void BSP_DMA2_Init(void);
void BSP_I2C2_Init(void);
void BSP_LED_Init(void);
void BSP_Button_Init(void);
void BSP_Beep_Init(void);
#endif
