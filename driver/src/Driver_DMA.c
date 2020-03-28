#include "Driver_DMA.h"

DMA_Type DMA_Table[8] = {{USART3, 1, DMA1_Stream3, DMA_IT_TCIF3, DMA_FLAG_TCIF3, DMA_FLAG_HTIF3},
                         {USART3, 0, DMA1_Stream1, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                         {USART6, 1, DMA2_Stream6, DMA_IT_TCIF6, DMA_FLAG_TCIF6, DMA_FLAG_HTIF6},
                         {USART6, 0, DMA2_Stream1, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                         {UART7, 1, DMA1_Stream1, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                         {UART7, 0, DMA1_Stream3, DMA_IT_TCIF3, DMA_FLAG_TCIF3, DMA_FLAG_HTIF3},
                         {UART8, 1, DMA1_Stream0, DMA_IT_TCIF0, DMA_FLAG_TCIF0, DMA_FLAG_HTIF0},
                         {UART8, 0, DMA1_Stream6, DMA_IT_TCIF6, DMA_FLAG_TCIF6, DMA_FLAG_HTIF6}};

void DMA_Select_Stream(DMA_Type *DMAx) {
    int i;

    for (i = 0; i < 7; i++) {
        if (DMAx->USARTx == DMA_Table[i].USARTx && DMAx->IS_SEND == DMA_Table[i].IS_SEND) {
            *DMAx = DMA_Table[i];
        }
    }
}

uint16_t DMA_Restart(USART_TypeDef *USARTx, uint8_t IS_SEND, uint16_t length) {
    DMA_Type DMAx;
    uint16_t len;

    // select stream
    DMAx.USARTx  = USARTx;
    DMAx.IS_SEND = IS_SEND;
    DMA_Select_Stream(&DMAx);

    // disable DMA
    DMA_Cmd(DMAx.DMAx_Streamy, DISABLE);
    while (DMA_GetFlagStatus(DMAx.DMAx_Streamy, DMAx.DMA_IT_TCIFx) != SET) {
    }
    len = length - DMA_GetCurrDataCounter(DMAx.DMAx_Streamy);

    // enable DMA
    DMA_ClearFlag(DMAx.DMAx_Streamy, DMAx.DMA_FLAG_TCIFx | DMAx.DMA_FLAG_HTIFx);
    while (DMA_GetCmdStatus(DMAx.DMAx_Streamy) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMAx.DMAx_Streamy, length);
    DMA_Cmd(DMAx.DMAx_Streamy, ENABLE);

    return len;
}