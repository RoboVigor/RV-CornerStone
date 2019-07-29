/**
 * @brief  中断服务函数根据地
 */

#include "interrupt.h"
#include "main.h"

// EXTI9_5 陀螺仪中断
void EXTI9_5_IRQHandler(void) {
    uint8_t suc;
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearFlag(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line8);
        Gyroscope_Update(&Gyroscope_EulerData);
    }
}

// DBus空闲中断(USART1)
void USART1_IRQHandler(void) {
    uint8_t UARTtemp;

    UARTtemp = USART1->DR;
    UARTtemp = USART1->SR;

    DMA_Cmd(DMA2_Stream2, DISABLE);

    //数据量正确
    if (DMA2_Stream2->NDTR == DBUS_BACK_LENGTH) {
        DBus_Update(&remoteData, &keyboardData, &mouseData, remoteBuffer); //解码
    }

    //重启DMA
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMA2_Stream2, DBUS_LENGTH + DBUS_BACK_LENGTH);
    DMA_Cmd(DMA2_Stream2, ENABLE);
}

/**
 * @brief USART3 串口中断
 * @note  视觉系统读取
 */
void USART3_IRQHandler(void) {
    uint8_t  tmp;
    uint16_t len;
    int      i;

    // clear IDLE flag
    tmp = USART3->DR;
    tmp = USART3->SR;

    // disable DMA and decode
    DMA_Cmd(DMA1_Stream1, DISABLE);
    while (DMA_GetFlagStatus(DMA1_Stream1, DMA_IT_TCIF1) != SET) {
    }
    len = Protocol_Buffer_Length - DMA_GetCurrDataCounter(DMA1_Stream1);
    for (i = 0; i < len; i++) {
        Protocol_Unpack(&Ps, Ps.receiveBuf[i]);
    }

    // enable DMA
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMA1_Stream1, Protocol_Buffer_Length);
    DMA_Cmd(DMA1_Stream1, ENABLE);
}

/**
 * @brief USART6 串口中断
 * @note  裁判系统读取
 */
void USART6_IRQHandler(void) {
    uint8_t  tmp;
    uint16_t len;
    int      i;

    // clear IDLE flag
    tmp = USART6->DR;
    tmp = USART6->SR;

    // disable DMA and decode
    DMA_Cmd(DMA2_Stream1, DISABLE);
    while (DMA_GetFlagStatus(DMA2_Stream1, DMA_IT_TCIF1) != SET) {
    }
    len = Protocol_Buffer_Length - DMA_GetCurrDataCounter(DMA2_Stream1);
    for (i = 0; i < len; i++) {
        Protocol_Unpack(&Judge, Judge.receiveBuf[i]);
    }

    // enable DMA
    DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMA2_Stream1, Protocol_Buffer_Length);
    DMA_Cmd(DMA2_Stream1, ENABLE);

    // Tx
    DMA_Cmd(DMA2_Stream6, DISABLE);
    DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6);
    while (DMA_GetCmdStatus(DMA2_Stream6) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMA2_Stream6, Protocol_Buffer_Length);
    DMA_Cmd(DMA2_Stream6, ENABLE);
}

/**
 * @brief UART7 串口中断
 */
void UART7_IRQHandler(void) {
    u8 res;

    if (USART_GetITStatus(UART7, USART_IT_RXNE) != RESET) { // 接收中断（必须以 0x0d 0x0a 结尾）
        res       = USART_ReceiveData(UART7);               // 读取数据
        UART7->DR = res;                                    // 输出数据
        RED_LIGHT_TOGGLE;
    }
}

/**
 * @brief UART8 串口中断
 */
void UART8_IRQHandler(void) {
    u8 res;

    if (USART_GetITStatus(UART8, USART_IT_RXNE) != RESET) { // 接收中断（必须以 0x0d 0x0a 结尾）
        res       = USART_ReceiveData(UART8);               // 读取数据
        UART8->DR = res;                                    // 输出数据
        RED_LIGHT_TOGGLE;
    }
}

// CAN1数据接收中断服务函数
void CAN1_RX0_IRQHandler(void) {
    CanRxMsg CanRxData;
    int      position;
    int      speed;

    // 读取数据
    CAN_Receive(CAN1, CAN_FIFO0, &CanRxData);
    position = (short) ((int) CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    speed    = (short) ((int) CanRxData.Data[2] << 8 | CanRxData.Data[3]);

    // 安排数据
    switch (CanRxData.StdId) {
    case 0x201:
        Motor_Update(&Motor_LF, position, speed);
        break;

    case 0x202:
        Motor_Update(&Motor_LB, position, speed);
        break;

    case 0x203:
        Motor_Update(&Motor_RB, position, speed);
        break;

    case 0x204:
        Motor_Update(&Motor_RF, position, speed);
        break;

    case 0x209:
        Motor_Update(&Motor_Yaw, position, speed);
        break;

    case 0x206:
        Motor_Update(&Motor_Pitch, position, 0);
        break;

    default:
        break;
    }
}

void CAN1_SCE_IRQHandler(void) {
    RED_LIGHT_ON;
    CAN_ClearITPendingBit(CAN1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR);
}

// CAN2数据接收中断服务函数
void CAN2_RX0_IRQHandler(void) {
    CanRxMsg CanRxData;
    int      position;
    int      speed;

    // 读取数据
    CAN_Receive(CAN2, CAN_FIFO0, &CanRxData);
    position = (short) ((int) CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    speed    = (short) ((int) CanRxData.Data[2] << 8 | CanRxData.Data[3]);

    //安排数据
    switch (CanRxData.StdId) {
    case 0x207:
        Motor_Update(&Motor_Stir, position, speed);
        break;
    default:
        break;
    }
}

// TIM2 高频计数器
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

void NMI_Handler(void) {
    printf("NMI_Handler");
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @return None
 */
void HardFault_Handler(void) {
    printf("HardFault_Handler");
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @return None
 */
void MemManage_Handler(void) {
    printf("MemManage_Handler");
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @return None
 */
void BusFault_Handler(void) {
    printf("BusFault_Handler");
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @return None
 */
void UsageFault_Handler(void) {
    printf("UsageFault_Handler");
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @return None
 */
void DebugMon_Handler(void) {
    printf("DebugMon_Handler");
}

// /**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @return None
//  */
// void SVC_Handler(void) {
//     //printf("SVC_Handler");
// }

// /**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @return None
//  */
// void PendSV_Handler(void) {
//     //printf("PendSV_Handler");
// }

// /**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @return None
//  */
// void SysTick_Handler(void) {
//     //printf("SysTick_Handler");
// }
