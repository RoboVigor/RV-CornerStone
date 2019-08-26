/**
 * @brief  中断服务函数根据地
 */

#include "main.h"
#include "interrupt.h"

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
 */
void USART3_IRQHandler(void) {
    u8 res;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) { // 接收中断（必须以 0x0d 0x0a 结尾）
        res = USART_ReceiveData(USART3);                     // 读取数据
        RED_LIGHT_TOGGLE;
    }

    if ((magic.sta & 0x8000) == 0) { // 接收未完成
        if (magic.sta & 0x4000) {    // 接收到 0x0d
            if (res != 0x0a)         // 接收错误，重新开始
                magic.sta = 0;
            else // 接收完成
                magic.sta |= 0x8000;
        } else { // 未接收到 0x0d
            if (res == 0x0d) {
                magic.sta |= 0x4000;
            } else {
                magic.buf[magic.sta & 0X3FFF] = res;
                magic.sta++;
                // USART3->DR = res;
                if (magic.sta > (MAGIC_MAX_LENGTH - 1)) magic.sta = 0; // 接收数据错误，重新开始接收
            }
        }
    }
}

/**
 * @brief USART6 串口中断
 */
void USART6_IRQHandler(void) {
    u8 res;

    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) { // 接收中断（必须以 0x0d 0x0a 结尾）
        res = USART_ReceiveData(USART6);                     // 读取数据
        RED_LIGHT_TOGGLE;
    }

    if ((magic.sta & 0x8000) == 0) { // 接收未完成
        if (magic.sta & 0x4000) {    // 接收到 0x0d
            if (res != 0x0a)         // 接收错误，重新开始
                magic.sta = 0;
            else // 接收完成
                magic.sta |= 0x8000;
        } else { // 未接收到 0x0d
            if (res == 0x0d) {
                magic.sta |= 0x4000;
            } else {
                magic.buf[magic.sta & 0X3FFF] = res;
                magic.sta++;
                // USART6->DR = res;
                if (magic.sta > (MAGIC_MAX_LENGTH - 1)) magic.sta = 0; // 接收数据错误，重新开始接收
            }
        }
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

    case 0x205:
        Motor_Update(&Motor_Upthrow1, position, speed);
        break;

    case 0x206:
        Motor_Update(&Motor_Upthrow2, position, speed);
        break;

    case 0x207:
        Motor_Update(&Motor_TH, position, speed);
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

    // 安排数据
    switch (CanRxData.StdId) {
    case 0x201:
        Motor_Update(&Motor_LGW, position, speed);
        break;

    case 0x202:
        Motor_Update(&Motor_RGW, position, speed);
        break;

    case 0x203:
        Motor_Update(&Motor_TV, position, speed);
        break;

    case 0x205:
        Motor_Update(&Motor_Rotate_Left, position, speed);
        break;

    case 0x206:
        Motor_Update(&Motor_Rotate_Right, position, speed);
        break;

        // case 0x206:
        //     Motor_Update(&Motor_Upthrow2, position, speed);
        //     break;

    default:
        break;
    }
}

// // TIM2 高频计数器
// extern volatile uint32_t ulHighFrequencyTimerTicks;

// void TIM2_IRQHandler(void) {
//     if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
//         ulHighFrequencyTimerTicks++;
//         TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//         TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//     }
// }

// TIM2 输入捕获初始化
u8 TIM2CH1_CAPTURE_STA = 0; //输入捕获状态

void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET) {
        if (TIM2CH1_CAPTURE_STA == 1) {
            TIM2CH1_CAPTURE_STA = 0;
            //获取当前的捕获值
            TIM2CH1_CAPTURE_VAL = TIM2->CCR1;
            //设置上升沿捕获
            TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Rising);
        } else {
            TIM2CH1_CAPTURE_STA = 1;
            TIM_SetCounter(TIM2, 0);                             //计数器清空
            TIM_OC1PolarityConfig(TIM2, TIM_ICPolarity_Falling); //设置下降沿捕获
        }
    }

    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1 | TIM_IT_Update); //清除中断标志位
}

u8 TIM3CH3_CAPTURE_STA = 0; //输入捕获状态

void TIM3_IRQHandler(void) {
    if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET) {
        if (TIM3CH3_CAPTURE_STA == 1) {
            TIM3CH3_CAPTURE_STA = 0;
            //获取当前的捕获值
            TIM3CH3_CAPTURE_VAL = TIM3->CCR3;
            //设置上升沿捕获
            TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Rising);
        } else {
            TIM3CH3_CAPTURE_STA = 1;
            TIM_SetCounter(TIM3, 0);                             //计数器清空
            TIM_OC3PolarityConfig(TIM3, TIM_ICPolarity_Falling); //设置下降沿捕获
        }
    }

    TIM_ClearITPendingBit(TIM3, TIM_IT_CC3 | TIM_IT_Update); //清除中断标志位
}

// TIM5 输入捕获初始化
u8 TIM5CH1_CAPTURE_STA = 0; //输入捕获状态

// TIM5 输入捕获
void TIM5_IRQHandler(void) {
    // 单通道输入捕获
    // 捕获 1 发生捕获事件
    if (TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) {
        // 捕获到一个下降沿
        if (TIM5CH1_CAPTURE_STA == 1) {
            TIM5CH1_CAPTURE_STA = 0;
            //获取当前的捕获值
            TIM5CH1_CAPTURE_VAL = TIM5->CCR1;
            //设置上升沿捕获
            TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Rising);
        } else {
            TIM5CH1_CAPTURE_STA = 1;
            TIM_SetCounter(TIM5, 0);                             //计数器清空
            TIM_OC1PolarityConfig(TIM5, TIM_ICPolarity_Falling); //设置下降沿捕获
        }
    }

    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1 | TIM_IT_Update); //清除中断标志位
}

void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    if (TIM_GetITStatus(TIM14, TIM_IT_Update) == SET) {
        if (Fsm_TIM14_State != 1) {
            Fsm_TIM14_Cnt = 0;
        } else {
            Fsm_TIM14_Cnt++;
        }
    }
    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
}

u8 TIM9CH1_CAPTURE_STA = 0;

void TIM1_BRK_TIM9_IRQHandler(void) {
    if (TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET) {
        // 捕获到一个下降沿
        if (TIM9CH1_CAPTURE_STA == 1) {
            TIM9CH1_CAPTURE_STA = 0;
            //获取当前的捕获值
            TIM9CH1_CAPTURE_VAL = TIM9->CCR1;
            //设置上升沿捕获
            TIM_OC1PolarityConfig(TIM9, TIM_ICPolarity_Rising);
        } else {
            TIM9CH1_CAPTURE_STA = 1;
            TIM_SetCounter(TIM9, 0);                             //计数器清空
            TIM_OC1PolarityConfig(TIM9, TIM_ICPolarity_Falling); //设置下降沿捕获
        }
    }

    TIM_ClearITPendingBit(TIM9, TIM_IT_CC1 | TIM_IT_Update); //清除中断标志位
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

    // disable DMA and Unpack
    DMA_Cmd(DMA1_Stream3, DISABLE);
    while (DMA_GetFlagStatus(DMA1_Stream3, DMA_IT_TCIF3) != SET) {
    }
    len = Protocol_Buffer_Length - DMA_GetCurrDataCounter(DMA1_Stream3);
    for (i = 0; i < len; i++) {
        Protocol_Unpack(&Board, Board.receiveBuf[i]);
    }

    // enable DMA
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE) {
    }
    DMA_SetCurrDataCounter(DMA1_Stream3, Protocol_Buffer_Length);
    DMA_Cmd(DMA1_Stream3, ENABLE);
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
