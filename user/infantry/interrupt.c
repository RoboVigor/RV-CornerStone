/**
 * @brief  中断服务函数根据地
 */

#include "handle.h"
#include "gimbal_tuning.h"
// EXTI9_5 陀螺仪中断
void EXTI9_5_IRQHandler(void) {
    uint8_t suc;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearFlag(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line8);
        xSemaphoreGiveFromISR(ImuDataReady, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// DBus空闲中断(USART1)
void USART1_IRQHandler(void) {
    uint8_t UARTtemp;

    UARTtemp = USART1->DR;
    UARTtemp = USART1->SR;

    DMA_Cmd(DMA2_Stream2, DISABLE);

    // disabe DMA
    DMA_Disable(USART1_Rx);

    // 数据量正确
    if (DMA_Get_Stream(USART1_Rx)->NDTR == DBUS_BACK_LENGTH) {
        DBus_Update(&remoteData, &keyboardData, &mouseData, remoteBuffer); //解码
    }

    // enable DMA
    DMA_Enable(USART1_Rx, DBUS_LENGTH + DBUS_BACK_LENGTH);
}

/**
 * @brief USART3 串口中断
 * @note  视觉系统读取
 */
void USART3_IRQHandler(void) {
#if GIMBAL_TUNING_ENABLED
    GimbalTuning_USART3_IRQHandler(); // ISR copies bytes; parsing is in control task.
#else
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 3);
#endif
}

/**
 * @brief USART6 串口中断
 * @note  裁判系统读取
 */
void USART6_IRQHandler(void) {
    //Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 6);
    Unitree_Receive(&Unitree_Bridge);
}

/**
 * @brief UART7 串口中断
 */
void UART7_IRQHandler(void) {
    Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 7);
}

/**
 * @brief UART8 串口中断
 */
void UART8_IRQHandler(void) {
   Bridge_Receive_USART(&BridgeData, USART_BRIDGE, 8);
}

// CAN1数据接收中断服务函数
void CAN1_RX0_IRQHandler(void) {
    Bridge_Receive_CAN(&BridgeData, CAN1_BRIDGE);
}

// CAN2数据接收中断服务函数
void CAN2_RX0_IRQHandler(void) {
    Bridge_Receive_CAN(&BridgeData, CAN2_BRIDGE);
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
    while (1) {
    }
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @return None
 */
void hardfault_c(uint32_t *sp);
	
__ASM void HardFault_Handler(void)
{
		IMPORT hardfault_c;
	
    TST     LR, #4
    ITE     EQ
    MRSEQ   R0, MSP
    MRSNE   R0, PSP
    B       hardfault_c
}

void hardfault_c(uint32_t *sp)
{
    volatile uint32_t pc  = sp[6];
    volatile uint32_t lr  = sp[5];
    volatile uint32_t cfsr  = SCB->CFSR;
    volatile uint32_t hfsr  = SCB->HFSR;
    volatile uint32_t mmfar = SCB->MMFAR;
    volatile uint32_t bfar  = SCB->BFAR;

    __BKPT(0);
    while (1) {}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @return None
 */
void MemManage_Handler(void) {
     while (1) {
    }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @return None
 */
void BusFault_Handler(void) {
   while (1) {
    }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @return None
 */
void UsageFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @return None
 */
void DebugMon_Handler(void) {
    while (1) {
    }
}

// /**
//  * @brief  This function handles SVCall exception.
//  * @param  None
//  * @return None
//  */
// void SVC_Handler(void) {
//     //while(1){}
// }

// /**
//  * @brief  This function handles PendSVC exception.
//  * @param  None
//  * @return None
//  */
// void PendSV_Handler(void) {
//    //while(1){}
// }

// /**
//  * @brief  This function handles SysTick Handler.
//  * @param  None
//  * @return None
//  */
// void SysTick_Handler(void) {
//      //while(1){}
// }
