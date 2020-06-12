#include "Driver_DMA.h"

DMA_Type DMA_Table[8] = {{USART3, TX, DMA1_Stream3, DMA_IT_TCIF3, DMA_FLAG_TCIF3, DMA_FLAG_HTIF3},
                         {USART3, RX, DMA1_Stream1, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                         {USART6, TX, DMA2_Stream6, DMA_IT_TCIF6, DMA_FLAG_TCIF6, DMA_FLAG_HTIF6},
                         {USART6, RX, DMA2_Stream1, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                         {UART7, TX, DMA1_Stream1, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                         {UART7, RX, DMA1_Stream3, DMA_IT_TCIF3, DMA_FLAG_TCIF3, DMA_FLAG_HTIF3},
                         {UART8, TX, DMA1_Stream0, DMA_IT_TCIF0, DMA_FLAG_TCIF0, DMA_FLAG_HTIF0},
                         {UART8, RX, DMA1_Stream6, DMA_IT_TCIF6, DMA_FLAG_TCIF6, DMA_FLAG_HTIF6}};

void DMA_Select_Stream(DMA_Type *DMAx) {
}

void DMA_Restart(USART_TypeDef *USARTx, trx_e TRX, Protocol_Type *Protocol, uint16_t id, uint16_t length) {
    DMA_Type DMA;
    uint16_t dataLength;
    int      i;
    uint8_t *send_p;

    // select stream
    DMA.USARTx = USARTx;
    DMA.TRX    = TRX;
    for (i = 0; i < 7; i++) {
        if (DMA.USARTx == DMA_Table[i].USARTx && DMA.TRX == DMA_Table[i].TRX) {
            DMA = DMA_Table[i];
        }
    }

    // disable DMA
    while (DMA_GetFlagStatus(DMA.DMAx_Streamy, DMA.DMA_IT_TCIFx) != SET) {
    }
    DMA_Cmd(DMA.DMAx_Streamy, DISABLE);

    // pack/unpack
    if (DMA.TRX == TX) {
        dataLength = length - PROTOCOL_HEADER_CRC_CMDID_LEN;
        Protocol_Pack(Protocol, dataLength, id);
        send_p = Protocol->sendBuf;
        for (i = 0; i < length; i++) {
            *send_p++ = Protocol->sendBuf[i];
        }
    } else {
        dataLength = length - DMA_GetCurrDataCounter(DMA.DMAx_Streamy);
        for (i = 0; i < length; i++) {
            Protocol_Unpack(Protocol, Protocol->receiveBuf[i]);
        }
    }

    // enable DMA
    DMA_ClearFlag(DMA.DMAx_Streamy, DMA.DMA_FLAG_TCIFx | DMA.DMA_FLAG_HTIFx);
    while (DMA_GetCmdStatus(DMA.DMAx_Streamy) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMA.DMAx_Streamy, length);
    DMA_Cmd(DMA.DMAx_Streamy, ENABLE);
}