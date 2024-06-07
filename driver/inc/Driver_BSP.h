/**
 * @file Driver_BSP.h
 * @brief BSP初始化
 */

#ifndef __DRIVER_BSP_H
#define __DRIVER_BSP_H

#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
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
#define PWM_PE9 0x11011109
#define PWM_PE11 0x1101210b
#define PWM_PE13 0x1101310d
#define PWM_PE14 0x1101410e
#define PWM_PC6 0x21431086

#define ADC_Channel0 0x00000001
#define ADC_Channel1 0x00000002
#define ADC_Channel2 0x00000004
#define ADC_Channel3 0x00000008
#define ADC_Channel4 0x00000010
#define ADC_Channel5 0x00000020
#define ADC_Channel6 0x00000040
#define ADC_Channel7 0x00000080
#define ADC_Channel8 0x00000100
#define ADC_Channel9 0x00000200
#define ADC_Channel10 0x00000400
#define ADC_Channel11 0x00000800
#define ADC_Channel12 0x00001000
#define ADC_Channel13 0x00002000
#define ADC_Channel14 0x00004000
#define ADC_Channel15 0x00008000
#define ADC_Channel16 0x00010000
#define ADC_Channel17 0x00020000
#define ADC_Channel18 0x00040000

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

typedef enum { Tx, Rx } trx_e;

typedef struct {
    uint32_t            PERIPHx_BASE;
    trx_e               TRx;
    DMA_TypeDef *       DMAx;
    DMA_Stream_TypeDef *DMAx_Streamy;
    uint32_t            DMAx_Streamy_IRQn;
    uint32_t            DMA_Channel_x;
    uint32_t            DMA_IT_TCIFx;
    uint32_t            DMA_FLAG_TCIFx;
    uint32_t            DMA_FLAG_HTIFx;
} DMA_Type;

typedef enum { USART1_Tx, USART1_Rx, USART3_Tx, USART3_Rx, USART6_Tx, USART6_Rx, UART7_Tx, UART7_Rx, UART8_Tx, UART8_Rx } dma_table_index_e;

#ifdef __BSP_GLOBALS
DMA_Type DMA_Table[10] = {{USART1_BASE, Tx, DMA2, DMA2_Stream7, DMA2_Stream7_IRQn, DMA_Channel_4, DMA_IT_TCIF7, DMA_FLAG_TCIF7, DMA_FLAG_HTIF7},
                          {USART1_BASE, Rx, DMA2, DMA2_Stream2, DMA2_Stream2_IRQn, DMA_Channel_4, DMA_IT_TCIF2, DMA_FLAG_TCIF2, DMA_FLAG_HTIF2},
                          {USART3_BASE, Tx, DMA1, DMA1_Stream3, DMA1_Stream3_IRQn, DMA_Channel_4, DMA_IT_TCIF3, DMA_FLAG_TCIF3, DMA_FLAG_HTIF3},
                          {USART3_BASE, Rx, DMA1, DMA1_Stream1, DMA1_Stream1_IRQn, DMA_Channel_4, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                          {USART6_BASE, Tx, DMA2, DMA2_Stream6, DMA2_Stream6_IRQn, DMA_Channel_5, DMA_IT_TCIF6, DMA_FLAG_TCIF6, DMA_FLAG_HTIF6},
                          {USART6_BASE, Rx, DMA2, DMA2_Stream1, DMA2_Stream1_IRQn, DMA_Channel_5, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                          {UART7_BASE, Tx, DMA1, DMA1_Stream1, DMA1_Stream1_IRQn, DMA_Channel_5, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                          {UART7_BASE, Rx, DMA1, DMA1_Stream3, DMA1_Stream3_IRQn, DMA_Channel_5, DMA_IT_TCIF3, DMA_FLAG_TCIF3, DMA_FLAG_HTIF3},
                          {UART8_BASE, Tx, DMA1, DMA1_Stream0, DMA1_Stream0_IRQn, DMA_Channel_5, DMA_IT_TCIF0, DMA_FLAG_TCIF0, DMA_FLAG_HTIF0},
                          {UART8_BASE, Rx, DMA1, DMA1_Stream6, DMA1_Stream6_IRQn, DMA_Channel_5, DMA_IT_TCIF6, DMA_FLAG_TCIF6, DMA_FLAG_HTIF6}};

#else
extern DMA_Type DMA_Table[10];
#endif

// Stone ID
void BSP_Stone_Id_Init(uint8_t *Board_Id, uint8_t *Robot_Id);

// SERVICE
void BSP_CAN_Init(void);
void BSP_DBUS_Init(uint8_t *remoteBuffer);
void BSP_IMU_Init(void);
void BSP_Laser_Init(void);
void BSP_User_Power_Init(void);

// 按键
void    BSP_Button_Init(void);
uint8_t Is_Button_Pressed(void);

// TEMPORARY USE
void BSP_TIM2_Init(void);

// USART
void BSP_USART_Init(
    USART_TypeDef *, uint32_t, uint8_t, uint8_t, uint8_t, uint16_t, GPIO_TypeDef *, uint16_t, uint32_t, uint16_t, uint16_t, uint16_t, uint32_t, uint16_t);
void BSP_USART2_Init(uint32_t baudRate, uint16_t interruptFlag);
void BSP_USART3_Init(uint32_t baudRate, uint16_t interruptFlag);
void BSP_USART6_Init(uint32_t baudRate, uint16_t interruptFlag);
void BSP_UART7_Init(uint32_t baudRate, uint16_t interruptFlag);
void BSP_UART8_Init(uint32_t baudRate, uint16_t interruptFlag);

// DMA
void                BSP_DMA_Init(dma_table_index_e tableIndex, uint32_t sourceMemoryAddress, uint32_t bufferSize);
void                DMA_Disable(dma_table_index_e tableIndex);
void                DMA_Enable(dma_table_index_e tableIndex, uint16_t length);
DMA_Stream_TypeDef *DMA_Get_Stream(dma_table_index_e tableIndex);
uint16_t            DMA_Get_Data_Counter(dma_table_index_e tableIndex);

// PWM
void BSP_PWM_Set_Port(PWM_Type *PWMx, uint32_t PWM_Px);
void BSP_PWM_Init(PWM_Type *PWMx, uint16_t prescaler, uint32_t period, uint16_t polarity);
void PWM_Set_Compare(PWM_Type *PWMx, uint32_t compare);

// LED
void BSP_LED_Init(void);
void LED_Set_Warning(uint16_t row, uint16_t blinkTimes);
void LED_Cancel_Warning();
void LED_Task_Warning();
void LED_Set_Row(uint16_t row);
void LED_Set_Progress(uint16_t progress);
void LED_Set_Colour(uint16_t red, uint16_t green, uint16_t blue);
void LED_Run_Horse();
void LED_Run_Horse_XP();
void LED_Run_Rainbow_Ball();

// BEEP
void    BSP_Beep_Init(void);
uint8_t KTV_Play(Song_Type song);

// OLED
void BSP_OLED_init(void);

// ADC
void BSP_ADC_Init(ADC_TypeDef *, uint16_t, uint32_t, uint32_t, uint32_t, uint16_t, uint16_t);
void BSP_ADC1_Init(uint32_t ADC_NbrOfConversion, uint32_t ADC_Channel, uint16_t interruptFlag);
void BSP_ADC2_Init(uint32_t ADC_NbrOfConversion, uint32_t ADC_Channel, uint16_t interruptFlag);
void BSP_ADC3_Init(uint32_t ADC_NbrOfConversion, uint32_t ADC_Channel, uint16_t interruptFlag);

#endif
