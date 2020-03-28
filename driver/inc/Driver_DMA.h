/**
 * @file Driver_DMA.h
 */

#ifndef __DRIVER_DMA_H
#define __DRIVER_DMA_H

#include "stm32f4xx.h"
#include "Driver_Protocol.h"

typedef struct {
    USART_TypeDef *     USARTx;
    uint8_t             IS_SEND; // 0:接收 1:发送
    DMA_Stream_TypeDef *DMAx_Streamy;
    uint32_t            DMA_IT_TCIFx;
    uint32_t            DMA_FLAG_TCIFx;
    uint32_t            DMA_FLAG_HTIFx;
} DMA_Type;

void     DMA_Select_Stream(DMA_Type *);
uint16_t DMA_Restart(USART_TypeDef *, uint8_t, uint16_t);

#endif
