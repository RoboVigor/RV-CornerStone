/**
 * @file Driver_DMA.h
 */

#ifndef __DRIVER_DMA_H
#define __DRIVER_DMA_H

#include "stm32f4xx.h"
#include "Driver_Protocol.h"

typedef enum {
    TX = 0,
    RX = 1,
} trx_e;

typedef struct {
    USART_TypeDef *     USARTx;
    trx_e               TRX;
    DMA_Stream_TypeDef *DMAx_Streamy;
    uint32_t            DMA_IT_TCIFx;
    uint32_t            DMA_FLAG_TCIFx;
    uint32_t            DMA_FLAG_HTIFx;
} DMA_Type;

void DMA_Restart(USART_TypeDef *USARTx, trx_e TRX, Protocol_Type *Protocol, uint16_t id, uint16_t length);

#endif
