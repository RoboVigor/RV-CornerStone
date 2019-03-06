/**
 * @brief  中断服务函数根据地
 */

#include "main.h"
#include "interrupt.h"

// DMA Handle function
void DMA2_Stream1_IRQHandler(void) {
    uint8_t UARTtemp;

    UARTtemp = USART6->DR;
    UARTtemp = USART6->SR;

    DMA_Cmd(DMA2_Stream1, DISABLE);
    if (DMA_GetFlagStatus(DMA2_Stream1, DMA_IT_TCIF1) != RESET) {
        Decode_JudgeData(); //½âÂë
    }
    //ÖØÆôDMA
    DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE)
        ;
    DMA_SetCurrDataCounter(DMA2_Stream1, JudgeBufferLength);
    DMA_Cmd(DMA2_Stream1, ENABLE);
}

// EXTI9_5 陀螺仪中断
void EXTI9_5_IRQHandler(void) //中断频率1KHz
{
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearFlag(EXTI_Line8);
        EXTI_ClearITPendingBit(EXTI_Line8);
        MPU6500_ReadData(MPU_IIC_ADDR, MPU6500_ACCEL_XOUT_H, mpu_buf, 14);
        mpu6500_data.ax   = (((int16_t) mpu_buf[0]) << 8) | mpu_buf[1];
        mpu6500_data.ay   = (((int16_t) mpu_buf[2]) << 8) | mpu_buf[3];
        mpu6500_data.az   = (((int16_t) mpu_buf[4]) << 8) | mpu_buf[5];
        mpu6500_data.temp = (((int16_t) mpu_buf[6]) << 8) | mpu_buf[7];
        mpu6500_data.gx   = (((int16_t) mpu_buf[8]) << 8) | mpu_buf[9];
        mpu6500_data.gx += mpu6500_data.gx_offset;
        mpu6500_data.gy = (((int16_t) mpu_buf[10]) << 8) | mpu_buf[11];
        mpu6500_data.gy += mpu6500_data.gy_offset;
        mpu6500_data.gz = (((int16_t) mpu_buf[12]) << 8) | mpu_buf[13];
        mpu6500_data.gz += mpu6500_data.gz_offset;

#if GYROSCOPE_YAW_START_UP_DELAY_ENABLED == 1
        if (Gyroscope_EulerData.downcounter <= GYROSCOPE_START_UP_DELAY) {
            Gyroscope_EulerData.downcounter = Gyroscope_EulerData.downcounter + 1;
        }
#endif
        Gyroscope_Update_Angle_Data();
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
 * @brief USART3 串口中断
 */
void USART3_IRQHandler(void) {
    u8 res;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) { // 接收中断（必须以 0x0d 0x0a 结尾）
        res = USART_ReceiveData(USART3);                     // 读取数据
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
                // USART3->DR = res;
                if (USART_RX_STA > (MAGIC_MAX_LENGTH - 1)) USART_RX_STA = 0; // 接收数据错误，重新开始接收
            }
        }
    }
}

/**
 * @brief USART2 串口中断
 */
void USART2_IRQHandler(void) {
    u8 res;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) { // 接收中断（必须以 0x0d 0x0a 结尾）
        res = USART_ReceiveData(USART2);                     // 读取数据
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
        Motor_Update(&Motor_Yaw, position, 0);
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

    // 安排数据
    // switch (CanRxData.StdId) {
    // case 0x201:
    //     Motor_Update(&Motor_LF, position, speed);
    //     break;

    // case 0x202:
    //     Motor_Update(&Motor_LB, position, speed);
    //     break;

    // case 0x203:
    //     Motor_Update(&Motor_RB, position, speed);
    //     break;

    // case 0x204:
    //     Motor_Update(&Motor_RF, position, speed);
    //     break;

    // default:
    //     break;
    // }
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
