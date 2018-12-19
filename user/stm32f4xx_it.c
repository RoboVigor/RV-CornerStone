/**
 * @brief  中断服务函数根据地
 */

#include "main.h"
#include "stm32f4xx_it.h"

/**
 * @brief  DBus空闲中断(USART1)
 * @param  void
 * @return void
 */

void USART1_IRQHandler(void) {
    uint8_t UARTtemp;

    UARTtemp = USART1->DR;
    UARTtemp = USART1->SR;

    DMA_Cmd(DMA2_Stream2, DISABLE);

    //数据量正确
    if (DMA2_Stream2->NDTR == DBUS_BACK_LENGTH) {
        DBus_Update(&remoteData, remoteBuffer); //解码
    }

    //重启DMA
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
    while (DMA_GetCmdStatus(DMA2_Stream2) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMA2_Stream2, DBUS_LENGTH + DBUS_BACK_LENGTH);
    DMA_Cmd(DMA2_Stream2, ENABLE);
}

/**
 * @brief  无线串口中断(USART6)
 * @param  void
 * @return void
 * 接收到s时往消息体里塞数据
 */

void USART6_IRQHandler(void) {
    u8 res;

    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) { // 接收中断（必须以 0x0d 0x0a 结尾）
        res = USART_ReceiveData(USART6);                     // 读取数据
        RED_LIGHT_TOGGLE;
    }
    if ((USART_RX_STA & 0x8000) == 0) { // 接收未完成
        if (USART_RX_STA & 0x4000) {    // 接收到 0x0d
            if (res != 0x0a)            // 接收错误，重新开始
                USART_RX_STA = 0;
            else // 接收完成
                USART_RX_STA |= 0x8000;
        } else { // 未接收到 0x0d
            if (res == 0x0d) {
                USART_RX_STA |= 0x4000;
            } else {
                USART_RX_BUF[USART_RX_STA & 0X3FFF] = res;
                USART_RX_STA++;
                // USART6->DR = res;
                if (USART_RX_STA > (MAGIC_MAX_LENGTH - 1)) USART_RX_STA = 0; // 接收数据错误，重新开始接收
            }
        }
    }
}

/**
 * @brief  CAN1数据接收中断服务函数
 * @param  void
 * @return void
 */

void CAN1_RX0_IRQHandler(void) {
    CanRxMsg CanRxData;
    int      position;
    int      angle;

    // 读取数据
    CAN_Receive(CAN1, CAN_FIFO0, &CanRxData);
    position = (short) ((int) CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    angle    = (short) ((int) CanRxData.Data[2] << 8 | CanRxData.Data[3]);

    // 安排数据
    switch (CanRxData.StdId) {
    case 0x201:
        Motor_Update(&Motor_LF, position, angle);
        break;

    case 0x202:
        Motor_Update(&Motor_LB, position, angle);
        break;

    case 0x203:
        Motor_Update(&Motor_RB, position, angle);
        break;

    case 0x204:
        Motor_Update(&Motor_RF, position, angle);
        break;

    case 0x207:
        Motor_Update(&Motor_Stir, position, angle);
        break;

    default:
        break;
    }
}

void CAN1_SCE_IRQHandler(void) {
    RED_LIGHT_ON;
    CAN_ClearITPendingBit(CAN1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR);
}

/**
 * @brief  CAN2数据接收中断服务函数
 * @param  void
 * @return void
 */

void CAN2_RX0_IRQHandler(void) {
    CanRxMsg CanRxData;
    int      position;
    int      angle;

    // 读取数据
    CAN_Receive(CAN2, CAN_FIFO0, &CanRxData);
    position = (short) ((int) CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    angle    = (short) ((int) CanRxData.Data[2] << 8 | CanRxData.Data[3]);

    // 安排数据
    switch (CanRxData.StdId) {
    case 0x201:
        Motor_Update(&Motor_LeftFrict, position, angle);
        break;

    case 0x202:
        Motor_Update(&Motor_RightFrict, position, angle);
        break;

        // Motor_Update(&Motor_Stir, position, angle);
        // break;

        // case 0x204:
        //     Motor_Update(&Motor_RF, position, angle);
        //     break;

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
