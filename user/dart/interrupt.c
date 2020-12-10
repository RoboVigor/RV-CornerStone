/**
 * @brief  中断服务函数根据地
 */

#include "interrupt.h"
#include "main.h"

/**
 * @brief EXTI4 加速度计中断
 */
void EXTI4_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
        EXTI_ClearFlag(EXTI_Line4);
        EXTI_ClearITPendingBit(EXTI_Line4);
        Gyroscope_Update(&Gyroscope_EulerData);
    }
}

/**
 * @brief EXTI9_5 陀螺仪中断
 */
void EXTI9_5_IRQHandler(void) {
#ifdef STM32F427_437xx
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearFlag(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line8);
        Gyroscope_Update(&Gyroscope_EulerData);
    }
#endif
#ifdef STM32F40_41xxx
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
        EXTI_ClearFlag(EXTI_Line5);
        EXTI_ClearITPendingBit(EXTI_Line5);
        Gyroscope_Update(&Gyroscope_EulerData);
    }
#endif
}

/**
 * @brief EXTI3 磁力计中断
 */
void EXTI3_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
        EXTI_ClearFlag(EXTI_Line3);
        EXTI_ClearITPendingBit(EXTI_Line3);
        Gyroscope_Update(&Gyroscope_EulerData);
    }
}

/**
 * @brief USART1 串口中断
 */
void USART1_IRQHandler(void) {
    uint8_t  tmp;
    uint16_t len;
    int      i;

    tmp = USART1->DR;
    tmp = USART1->SR;

#ifdef STM32F427_437xx
    // disabe DMA
    DMA_Disable(USART1_Rx);

    //数据量正确
    if (DMA_Get_Stream(USART1_Rx)->NDTR == DBUS_BACK_LENGTH) {
        DBus_Update(&remoteData, &keyboardData, &mouseData, remoteBuffer); //解码
    }

    // enable DMA
    DMA_Enable(USART1_Rx, DBUS_LENGTH + DBUS_BACK_LENGTH);
#endif
}

/**
 * @brief USART3
 */
void USART3_IRQHandler(void) {
    uint8_t tmp;

    // clear IDLE flag
    tmp = USART3->DR;
    tmp = USART3->SR;

    // disabe DMA
    DMA_Disable(USART3_Rx);

    //数据量正确
    if (DMA_Get_Stream(USART3_Rx)->NDTR == DBUS_BACK_LENGTH) {
        DBus_Update(&remoteData, &keyboardData, &mouseData, remoteBuffer); //解码
    }

    // enable DMA
    DMA_Enable(USART3_Rx, DBUS_LENGTH + DBUS_BACK_LENGTH);
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
}

// void CAN1_SCE_IRQHandler(void) {
//     RED_LIGHT_ON;
//     CAN_ClearITPendingBit(CAN1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR);
// }

// CAN2数据接收中断服务函数
void CAN2_RX0_IRQHandler(void) {
    CanRxMsg CanRxData;
    int      position;
    int      speed;

    // 读取数据
    CAN_Receive(CAN2, CAN_FIFO0, &CanRxData);
    position = (short) ((int) CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    speed    = (short) ((int) CanRxData.Data[2] << 8 | CanRxData.Data[3]);
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
