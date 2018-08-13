/**
 * @brief  中断服务函数根据地
 * @rule   不应在此(包括头文件)声明任何全局变量
 */

#include "main.h"
#include "stm32f4xx_it.h"

/**
 * @brief  DBUS空闲中断(USART1)
 * @param  void
 * @return void
 */

uint8_t    UARTtemp;
BaseType_t xHigherPriorityTaskWoken;

void USART1_IRQHandler(void) {

    UARTtemp = USART1->DR;
    UARTtemp = USART1->SR;

    DMA_Cmd(DMA2_Stream2, DISABLE);

    //数据量正确
    if (DMA2_Stream2->NDTR == DBUSBackLength) {
        DBUS_DataDecoding(); //解码
    }

    //重启DMA
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMA2_Stream2, DBUSLength + DBUSBackLength);
    DMA_Cmd(DMA2_Stream2, ENABLE);
}

/**
 * @brief  无线串口中断(USART6)
 * @param  void
 * @return void
 */

void USART6_IRQHandler(void) {
    u8 Res;
    Res = USART_ReceiveData(USART6);
    USART_SendData(USART6, Res);
}
/**
 * @brief  CAN1数据接收中断服务函数
 * @param  void
 * @return void
 */

CanRxMsg CanRxData;

void CAN1_RX0_IRQHandler(void) {
    CAN_Receive(CAN1, CAN_FIFO0, &CanRxData);
    switch (CanRxData.StdId) {
    case WHEEL_1_ID:
        Motor_Feedback.Motor_201_Agree =
            (short) ((int) CanRxData.Data[ 0 ] << 8 | CanRxData.Data[ 1 ]);
        Motor_Feedback.Motor_201_Speed =
            (short) ((int) CanRxData.Data[ 2 ] << 8 | CanRxData.Data[ 3 ]);
        break;

    case WHEEL_2_ID:
        Motor_Feedback.Motor_202_Agree =
            (short) ((int) CanRxData.Data[ 0 ] << 8 | CanRxData.Data[ 1 ]);
        Motor_Feedback.Motor_202_Speed =
            (short) ((int) CanRxData.Data[ 2 ] << 8 | CanRxData.Data[ 3 ]);
        break;

    case WHEEL_3_ID:
        Motor_Feedback.Motor_203_Agree =
            (short) ((int) CanRxData.Data[ 0 ] << 8 | CanRxData.Data[ 1 ]);
        Motor_Feedback.Motor_203_Speed =
            (short) ((int) CanRxData.Data[ 2 ] << 8 | CanRxData.Data[ 3 ]);
        break;

    case WHEEL_4_ID:
        Motor_Feedback.Motor_204_Agree =
            (short) ((int) CanRxData.Data[ 0 ] << 8 | CanRxData.Data[ 1 ]);
        Motor_Feedback.Motor_204_Speed =
            (short) ((int) CanRxData.Data[ 2 ] << 8 | CanRxData.Data[ 3 ]);
        break;

    default:
        break;
    }
}

/**
 * @brief  CAN2数据接收中断服务函数
 * @param  void
 * @return void
 */

void CAN2_RX0_IRQHandler(void) {
    CAN_Receive(CAN2, CAN_FIFO0, &CanRxData);
    // printf("%d\r\n",CanRxData.StdId);
    switch (CanRxData.StdId) {
    // RED_LIGHT_ON;
    case 0x201: //钩子电机
        Motor_Feedback.Motor_205_Agree =
            (short) ((int) CanRxData.Data[ 0 ] << 8 | CanRxData.Data[ 1 ]);
        Motor_Feedback.Motor_205_Speed =
            (short) ((int) CanRxData.Data[ 2 ] << 8 | CanRxData.Data[ 3 ]);
        break;

    case 0x202:

        Motor_Feedback.Motor_206_Agree =
            (short) ((int) CanRxData.Data[ 0 ] << 8 | CanRxData.Data[ 1 ]);
        Motor_Feedback.Motor_206_Speed =
            (short) ((int) CanRxData.Data[ 2 ] << 8 | CanRxData.Data[ 3 ]);

        break;

    case 0x203:
        Motor_Feedback.Motor_207_Agree =
            (short) ((int) CanRxData.Data[ 0 ] << 8 | CanRxData.Data[ 1 ]);
        Motor_Feedback.Motor_207_Speed =
            (short) ((int) CanRxData.Data[ 2 ] << 8 | CanRxData.Data[ 3 ]);

        break;

    default:
        break;
    }
}

/**
 * @brief  TIM2 高频计数器
 * @param  void
 * @return void
 */

extern volatile uint32_t ulHighFrequencyTimerTicks;

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        ulHighFrequencyTimerTicks++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }
}

/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @return None
 */

void NMI_Handler(void) {}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @return None
 */
void HardFault_Handler(void) {
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @return None
 */
void MemManage_Handler(void) {
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @return None
 */
void BusFault_Handler(void) {
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @return None
 */
void UsageFault_Handler(void) {
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) {
    }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @return None
 */
// void SVC_Handler(void)
//{
//}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @return None
 */
void DebugMon_Handler(void) {}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @return None
 */
// void PendSV_Handler(void)
//{
//}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @return None
 */
// void SysTick_Handler(void)
//{
//
//}
