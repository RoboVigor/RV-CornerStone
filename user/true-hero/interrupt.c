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

    // disabe DMA
    DMA_Disable(USART1_Rx);

    //数据量正确
    if (DMA_Get_Stream(USART1_Rx)->NDTR == DBUS_BACK_LENGTH) {
        DBus_Update(&remoteData, &keyboardData, &mouseData, remoteBuffer); //解码
    }

    // enable DMA
    DMA_Enable(USART1_Rx, DBUS_LENGTH + DBUS_BACK_LENGTH);
}

/**
 * @brief USART3 串口中断
 */
void USART3_IRQHandler(void) {
    uint8_t tmp;

    // clear IDLE flag
    tmp = USART3->DR;
    tmp = USART3->SR;
}

/**
 * @brief USART6 串口中断
 */
void USART6_IRQHandler(void) {
    uint8_t  tmp;
    uint16_t len;
    int      i;

    // clear IDLE flag
    tmp = USART6->DR;
    tmp = USART6->SR;

    // disabe DMA
    DMA_Disable(USART6_Rx);

    // unpack
    len = Protocol_Buffer_Length - DMA_Get_Data_Counter(USART6_Rx);
    for (i = 0; i < len; i++) {
        Protocol_Unpack(&JudgeChannel, JudgeChannel.receiveBuf[i]);
    }

    // enable DMA
    DMA_Enable(USART6_Rx, Protocol_Buffer_Length);
}

/**
 * @brief UART7 串口中断
 */
void UART7_IRQHandler(void) {
    uint8_t  tmp;
    uint16_t len;
    int      i;

    // clear IDLE flag
    tmp = UART7->DR;
    tmp = UART7->SR;

    // disabe DMA
    DMA_Disable(UART7_Rx);

    // unpack
    len = Protocol_Buffer_Length - DMA_Get_Data_Counter(UART7_Rx);
    for (i = 0; i < len; i++) {
        Protocol_Unpack(&UserChannel, UserChannel.receiveBuf[i]);
    }

    // enable DMA
    DMA_Enable(UART7_Rx, Protocol_Buffer_Length);
}

/**
 * @brief UART8 串口中断
 */
void UART8_IRQHandler(void) {
    uint8_t  tmp;
    uint16_t len;
    int      i;

    // clear IDLE flag
    tmp = UART8->DR;
    tmp = UART8->SR;

    // disabe DMA
    DMA_Disable(UART8_Rx);

    // unpack
    len = Protocol_Buffer_Length - DMA_Get_Data_Counter(UART8_Rx);
    for (i = 0; i < len; i++) {
        Protocol_Unpack(&HostChannel, HostChannel.receiveBuf[i]);
    }

    // enable DMA
    DMA_Enable(UART8_Rx, Protocol_Buffer_Length);
}

// CAN1数据接收中断服务函数
void CAN1_RX0_IRQHandler(void) {
    CanRxMsg CanRxData;
    int      i;

    // 读取数据
    CAN_Receive(CAN1, CAN_FIFO0, &CanRxData);

    // 安排数据
    if (CanRxData.StdId < 0x500) {
        Motor_Update(Can1_Device[ESC_ID(CanRxData.StdId)], CanRxData.Data);
    } else {
        for (i = 0; i < 8; i++) {
            Protocol_Unpack(&UserChannel, CanRxData.Data[i]);
        }
    }
}

// void CAN1_SCE_IRQHandler(void) {
//     RED_LIGHT_ON;
//     CAN_ClearITPendingBit(CAN1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR);
// }

// CAN2数据接收中断服务函数
void CAN2_RX0_IRQHandler(void) {
    CanRxMsg CanRxData;
    int      data[8];

    // 读取数据
    CAN_Receive(CAN2, CAN_FIFO0, &CanRxData);

    //安排数据
    Motor_Update(Can2_Device[ESC_ID(CanRxData.StdId)], CanRxData.Data);
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
