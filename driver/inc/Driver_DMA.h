/**
 * @file Driver_DMA.h
 */

#ifndef __DRIVER_DMA_H
#define __DRIVER_DMA_H

#include "stm32f4xx.h"
#include "stddef.h"
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

/**
 * @brief DMA重启和解包（打包）
 *
 * @param USARTx 指定一个USART口,USART3/6,UART7/8
 * @param TRX 发送接收标志位
 * @param Protocol 通讯协议
 * @param id 发送时打包所需的id
 * @param length 缓存区长度
 */
void DMA_Restart(USART_TypeDef *USARTx, trx_e TRX, Protocol_Type *Protocol, uint16_t id, uint16_t length);

#endif
