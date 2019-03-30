/**
 * @brief  中断服务函数根据地
 */

#include "main.h"
#include "interrupt.h"

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
        Gyroscope_Update_Angle_Data(&Gyroscope_EulerData);
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

// 传感器串口中断配置
void USART2_IRQHandler(void) {
    static uint8_t i = 0, recevie_data[20] = {0};

    if (USART_GetFlagStatus(USART2, USART_FLAG_PE) != RESET) {
        USART_ReceiveData(USART2);
        USART_ClearFlag(USART2, USART_FLAG_PE);
    }

    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET) {
        USART_ReceiveData(USART2);
        USART_ClearFlag(USART2, USART_FLAG_ORE);
    }

    if (USART_GetFlagStatus(USART2, USART_FLAG_FE) != RESET) {
        USART_ReceiveData(USART2);
        USART_ClearFlag(USART2, USART_FLAG_FE);
    }

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //判断接收标志
    {
        USART_ClearFlag(USART2, USART_FLAG_RXNE);       // 试图解决问题
        USART_ClearITPendingBit(USART2, USART_IT_RXNE); // 试图解决问题

        recevie_data[i++] = USART_ReceiveData(USART2); //读取串口数据，同时清接收标志
        if (recevie_data[0] != 0x5a)                   //帧头不对
            i = 0;
        if ((i == 2) && (recevie_data[1] != 0x5a)) //帧头不对
            i = 0;

        if (i > 3) // i等于4时，已经接收到数据量字节recevie_data[3]
        {
            if (i != (recevie_data[3] + 5)) //判断是否接收一帧数据完毕
                return;
            switch (recevie_data[2]) //接收完毕后处理
            {
            case 0x15:
                if (!receive_ok) //当数据处理完成后才接收新的数据
                {
                    memcpy(re_buf_Data, recevie_data, 8); //拷贝接收到的数据
                    receive_ok = 1;                       //接收完成标志
                }
                break;
            }
            i = 0; //缓存清0
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
        Motor_Update(&Motor_TakeLeft, position, speed);
        break;

    case 0x206:
        Motor_Update(&Motor_TakeRight, position, speed);
        break;

    case 0x207:
        Motor_Update(&Motor_Transmission, position, speed);
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

        // case 0x203:
        //     Motor_Update(&Motor_RB, position, speed);
        //     break;

        // case 0x204:
        //     Motor_Update(&Motor_RF, position, speed);
        //     break;

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
 * @TIM5 输入捕获
 * 捕获状态
 * [7]:0,没有成功的捕获;1,成功捕获到一次. 
 * [6]:0,还没捕获到低电平;1,已经捕获到低电平了. 
 * [5:0]:捕获低电平后溢出的次数(对于 32 位定时器来说,1us 计数器加 1,溢出时间:4294 秒)
 */
// TIM5 输入捕获初始化
u8  TIM5CH1_CAPTURE_STA=0; //输入捕获状态 
u32 TIM5CH1_CAPTURE_VAL; //输入捕获值(TIM2/TIM5 是 32 位) 

void TIM5_IRQHandler(void)  {
    // 还未成功捕获
    if((TIM5CH1_CAPTURE_STA&0X80)==0) {
        // 溢出(在此应用中不会溢出)
        // if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
        //     // 已经捕获到高电平了 
        //     if(TIM5CH1_CAPTURE_STA&0X40) {
        //         // 高电平太长
        //         if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F) {
        //             TIM5CH1_CAPTURE_STA|=0X80; // 标记成功捕获一次
        //             TIM5CH1_CAPTURE_VAL=0XFFFFFFFF; 
        //         } else {
        //             TIM5CH1_CAPTURE_STA++; 
        //         }
        //     }
        // }
        // 捕获 1 发生捕获事件 
        if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET) {
            // 捕获到一个下降沿 
            if(TIM5CH1_CAPTURE_STA&0X40) {
                //标记成功捕获到一次高电平脉宽 
                TIM5CH1_CAPTURE_STA|=0X80;
                //获取当前的捕获值
                TIM5CH1_CAPTURE_VAL=TIM_GetCapture1(TIM5);
                //设置上升沿捕获 
                TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); 
            } else {
                TIM5CH1_CAPTURE_STA=0; // 清空
                TIM5CH1_CAPTURE_VAL=0;
                TIM5CH1_CAPTURE_STA|=0X40; //标记捕获到了上升沿 
                TIM_Cmd(TIM5, ENABLE);  //使能定时器 5 
                TIM_SetCounter(TIM5,0); //计数器清空
                TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);//设置下降沿捕获 
                TIM_Cmd(TIM5,ENABLE );  //使能定时器 5
            }

        }
    }
    
    TIM_ClearITPendingBit(TIM5, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位 
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
