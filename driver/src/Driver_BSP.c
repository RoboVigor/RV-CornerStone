#include "Driver_BSP.h"
#include "Driver_DBUS.h"
#include "math.h"
#include "macro.h"

void BSP_CAN_Init(void) {
    // GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    // CAN1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
    // CAN2
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    // CAN
    CAN_InitTypeDef       CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
    // CAN1
    CAN_InitStructure.CAN_ABOM      = DISABLE;
    CAN_InitStructure.CAN_AWUM      = DISABLE;
    CAN_InitStructure.CAN_BS1       = CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2       = CAN_BS2_5tq;
    CAN_InitStructure.CAN_Mode      = CAN_Mode_Normal;
    CAN_InitStructure.CAN_NART      = DISABLE;
    CAN_InitStructure.CAN_Prescaler = 3;
    CAN_InitStructure.CAN_RFLM      = DISABLE;
    CAN_InitStructure.CAN_SJW       = CAN_SJW_1tq;
    CAN_InitStructure.CAN_TTCM      = DISABLE;
    CAN_InitStructure.CAN_TXFP      = DISABLE;
    CAN_Init(CAN1, &CAN_InitStructure);
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterNumber         = 0;
    CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
    CAN_FilterInit(&CAN_FilterInitStructure);
    // CAN2
    CAN_InitStructure.CAN_ABOM      = ENABLE;
    CAN_InitStructure.CAN_AWUM      = DISABLE;
    CAN_InitStructure.CAN_BS1       = CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2       = CAN_BS2_5tq;
    CAN_InitStructure.CAN_Mode      = CAN_Mode_Normal;
    CAN_InitStructure.CAN_NART      = DISABLE;
    CAN_InitStructure.CAN_Prescaler = 3;
    CAN_InitStructure.CAN_RFLM      = DISABLE;
    CAN_InitStructure.CAN_SJW       = CAN_SJW_1tq;
    CAN_InitStructure.CAN_TTCM      = DISABLE;
    CAN_InitStructure.CAN_TXFP      = DISABLE;
    CAN_Init(CAN2, &CAN_InitStructure);
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterNumber         = 14;
    CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
    CAN_FilterInit(&CAN_FilterInitStructure);

    // NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    // CAN1
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
    // CAN2
    NVIC_InitStructure.NVIC_IRQChannel                   = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}

void BSP_USART_Init(USART_TypeDef *USARTx,
                    uint32_t       RCC_AHBxPeriph_GPIOx,
                    uint8_t        GPIO_AF_USARTx,
                    uint8_t        GPIO_PinSourcex,
                    uint8_t        GPIO_PinSourcey,
                    uint16_t       GPIO_Pin,
                    GPIO_TypeDef * GPIOx,
                    uint16_t       RCC_APBx,
                    uint32_t       RCC_APBxPeriph_USARTx,
                    uint16_t       USART_Mode,
                    uint16_t       USARTx_IRQn,
                    uint16_t       priority,
                    uint32_t       baudRate,
                    uint16_t       interruptFlag) {
    // GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHBxPeriph_GPIOx, ENABLE);
    GPIO_PinAFConfig(GPIOx, GPIO_PinSourcex, GPIO_AF_USARTx); // GPIO复用
    GPIO_PinAFConfig(GPIOx, GPIO_PinSourcey, GPIO_AF_USARTx); // GPIO复用
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin;                 // GPIO复用
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;             // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;         // 速度50MHz todo:测试2MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;            // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;             // 上拉
    GPIO_Init(GPIOx, &GPIO_InitStructure);                    // 初始化
    // USART
    if (RCC_APBx == RCC_APB1)
        RCC_APB1PeriphClockCmd(RCC_APBxPeriph_USARTx, ENABLE); // 使能时钟
    else if (RCC_APBx == RCC_APB2)
        RCC_APB2PeriphClockCmd(RCC_APBxPeriph_USARTx, ENABLE); // 使能时钟
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = baudRate;                       // 波特率设置
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_InitStructure.USART_Mode                = USART_Mode;                     // 收发模式
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;            // 字长为8位数据格式
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;               // 一个停止位
    USART_InitStructure.USART_Parity              = USART_Parity_No;                // 无奇偶校验位
    USART_Init(USARTx, &USART_InitStructure);                                       // 初始化串口
    USART_Cmd(USARTx, ENABLE);                                                      // 使能串口
    // NVIC
    if (interruptFlag != 0) {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel                   = USARTx_IRQn; // 串口中断通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;    // 抢占优先级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;           // 子优先级
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;      // IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);                                     // 根据指定的参数初始化VIC寄存器
        USART_ITConfig(USARTx, interruptFlag, ENABLE);                      // 开启相关中断
    }
}

void BSP_DBUS_Init(uint8_t *remoteBuffer) {
    // USART
    BSP_USART_Init(USART1,
                   RCC_AHB1Periph_GPIOB,
                   GPIO_AF_USART1,
                   GPIO_PinSource7,
                   GPIO_PinSource7,
                   GPIO_Pin_7,
                   GPIOB,
                   RCC_APB2,
                   RCC_APB2Periph_USART1,
                   USART_Mode_Rx,
                   USART1_IRQn,
                   8,
                   100000,
                   USART_IT_IDLE);
    // DMA
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)(remoteBuffer);
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = DBUS_LENGTH + DBUS_BACK_LENGTH;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream2, ENABLE);
}

/**
 * @brief USART初始化
 *
 * @param baudRate      波特率
 * @param interruptFlag 无中断   0
 *                      接收中断 USART_IT_RXNE
 *                      空闲中断 USART_IT_IDLE
 */
void BSP_USART2_Init(uint32_t baudRate, uint16_t interruptFlag) {
    BSP_USART_Init(USART2,
                   RCC_AHB1Periph_GPIOD,
                   GPIO_AF_USART2,
                   GPIO_PinSource5,
                   GPIO_PinSource6,
                   GPIO_Pin_5 | GPIO_Pin_6,
                   GPIOD,
                   RCC_APB1,
                   RCC_APB1Periph_USART2,
                   USART_Mode_Tx | USART_Mode_Rx,
                   USART2_IRQn,
                   2,
                   baudRate,
                   interruptFlag);
}

/**
 * @brief USART初始化
 *
 * @param baudRate      波特率
 * @param interruptFlag 无中断   0
 *                      接收中断 USART_IT_RXNE
 *                      空闲中断 USART_IT_IDLE
 */
void BSP_USART3_Init(uint32_t baudRate, uint16_t interruptFlag) {
    BSP_USART_Init(USART3,
                   RCC_AHB1Periph_GPIOD,
                   GPIO_AF_USART3,
                   GPIO_PinSource8,
                   GPIO_PinSource9,
                   GPIO_Pin_8 | GPIO_Pin_9,
                   GPIOD,
                   RCC_APB1,
                   RCC_APB1Periph_USART3,
                   USART_Mode_Tx | USART_Mode_Rx,
                   USART3_IRQn,
                   2,
                   baudRate,
                   interruptFlag);
}

/**
 * @brief USART初始化
 *
 * @param baudRate      波特率
 * @param interruptFlag 无中断   0
 *                      接收中断 USART_IT_RXNE
 *                      空闲中断 USART_IT_IDLE
 */
void BSP_USART6_Init(uint32_t baudRate, uint16_t interruptFlag) {
    BSP_USART_Init(USART6,
                   RCC_AHB1Periph_GPIOG,
                   GPIO_AF_USART6,
                   GPIO_PinSource9,
                   GPIO_PinSource14,
                   GPIO_Pin_9 | GPIO_Pin_14,
                   GPIOG,
                   RCC_APB2,
                   RCC_APB2Periph_USART6,
                   USART_Mode_Tx | USART_Mode_Rx,
                   USART6_IRQn,
                   2,
                   baudRate,
                   interruptFlag);
}

/**
 * @brief USART初始化
 *
 * @param baudRate      波特率
 * @param interruptFlag 无中断   0
 *                      接收中断 USART_IT_RXNE
 *                      空闲中断 USART_IT_IDLE
 */
void BSP_UART7_Init(uint32_t baudRate, uint16_t interruptFlag) {
    BSP_USART_Init(UART7,
                   RCC_AHB1Periph_GPIOE,
                   GPIO_AF_UART7,
                   GPIO_PinSource7,
                   GPIO_PinSource8,
                   GPIO_Pin_7 | GPIO_Pin_8,
                   GPIOE,
                   RCC_APB1,
                   RCC_APB1Periph_UART7,
                   USART_Mode_Tx | USART_Mode_Rx,
                   UART7_IRQn,
                   2,
                   baudRate,
                   interruptFlag);
}

/**
 * @brief USART初始化
 *
 * @param baudRate      波特率
 * @param interruptFlag 无中断   0
 *                      接收中断 USART_IT_RXNE
 *                      空闲中断 USART_IT_IDLE
 */
void BSP_UART8_Init(uint32_t baudRate, uint16_t interruptFlag) {
    BSP_USART_Init(UART8,
                   RCC_AHB1Periph_GPIOE,
                   GPIO_AF_UART8,
                   GPIO_PinSource0,
                   GPIO_PinSource1,
                   GPIO_Pin_0 | GPIO_Pin_1,
                   GPIOE,
                   RCC_APB1,
                   RCC_APB1Periph_UART8,
                   USART_Mode_Tx | USART_Mode_Rx,
                   UART8_IRQn,
                   7,
                   baudRate,
                   interruptFlag);
}

void BSP_ADC_Init(ADC_TypeDef *ADCx,
                  uint16_t     RCC_APBx,
                  uint32_t     RCC_APBxPeriph_ADCx,
                  uint32_t     ADC_NbrOfConversion,
                  uint32_t     ADC_Channel,
                  uint16_t     priority,
                  uint16_t     interruptFlag) {
    uint8_t               ADC_Channelx;
    uint8_t               Rank = 1;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;

    if (RCC_APBx == RCC_APB1)
        RCC_APB1PeriphClockCmd(RCC_APBxPeriph_ADCx, ENABLE); // 使能时钟
    else if (RCC_APBx == RCC_APB2)
        RCC_APB2PeriphClockCmd(RCC_APBxPeriph_ADCx, ENABLE); // 使能时钟

    // ADC通用配置
    ADC_CommonInitStructure.ADC_Mode             = ADC_Mode_Independent;         // 独立模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // 两个采样阶段之间的延迟x个时钟
    ADC_CommonInitStructure.ADC_DMAAccessMode    = ADC_DMAAccessMode_Disabled;   // DMA使能（DMA传输下要设置使能）
    ADC_CommonInitStructure.ADC_Prescaler        = ADC_Prescaler_Div2;           // 预分频4分频
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADCx配置
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;           // 右对齐
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConvEdge_None; // 使用软件触发（暂定）
    ADC_InitStructure.ADC_NbrOfConversion    = ADC_NbrOfConversion;           // 转换数量
    ADC_InitStructure.ADC_Resolution         = ADC_Resolution_12b;            // 12位模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                        // 开启连续转换（开启DMA传输要设置连续转换）
    ADC_InitStructure.ADC_ScanConvMode       = ENABLE;                        // 扫描（开启DMA传输要设置扫描）
    ADC_Init(ADCx, &ADC_InitStructure);

    for (ADC_Channelx = ADC_Channel_0; ADC_Channelx <= ADC_Channel_18; ADC_Channelx++) {
        if (ADC_Channel >> ADC_Channelx & 0x01 == 1) {
            ADC_RegularChannelConfig(ADCx, ADC_Channelx, Rank, ADC_SampleTime_3Cycles);
            Rank++;
        }
    }

    ADC_Cmd(ADCx, ENABLE);

    // NVIC
    if (interruptFlag != 0) {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel                   = ADC_IRQn; // 串口中断通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority; // 抢占优先级
        NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;        // 子优先级
        NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;   // IRQ通道使能
        NVIC_Init(&NVIC_InitStructure);                                  // 根据指定的参数初始化VIC寄存器
    }
}

/**
 * @brief ADC1初始化
 * @param ADC_NbrOfConversion 转换数量
 * @param ADC_Channel 通道选择
 * @param interruptFlag 有无中断
 */
void BSP_ADC1_Init(uint32_t ADC_NbrOfConversion, uint32_t ADC_Channel, uint16_t interruptFlag) {
    BSP_ADC_Init(ADC1, RCC_APB2, RCC_APB2Periph_ADC1, ADC_NbrOfConversion, ADC_Channel, 2, interruptFlag);
}

void BSP_Laser_Init(void) {
    // Laser
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOG, &GPIO_InitStructure);
}

void BSP_User_Power_Init(void) {
    // 24V User Power
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOH, &GPIO_InitStructure);
    GPIO_SetBits(GPIOH, GPIO_Pin_2);
    GPIO_SetBits(GPIOH, GPIO_Pin_3);
    GPIO_SetBits(GPIOH, GPIO_Pin_4);
    GPIO_SetBits(GPIOH, GPIO_Pin_5);
}

void BSP_IMU_Init(void) {
    // IIC
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    // IMU中断
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // EXTI
    EXTI_InitTypeDef EXTI_InitStructure;
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, GPIO_PinSource8);
    EXTI_InitStructure.EXTI_Line    = EXTI_Line8;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

void BSP_TIM2_Init(void) {
    // GPIO
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2); // GPIOA1复用为定时器2
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;             // GPIOB4
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;           //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;      //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;          //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;           //上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);                  //初始化
    // TIM
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TIM 频率
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitStructure.TIM_Period        = 2 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     = 9000 - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    TIM_Cmd(TIM2, ENABLE);
    // NVIC
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}

DMA_Type DMA_Table[10] = {{USART1_BASE, Tx, DMA2, DMA2_Stream7, DMA2_Stream7_IRQn, DMA_Channel_4, DMA_IT_TCIF7, DMA_FLAG_TCIF7, DMA_FLAG_HTIF7},
                          {USART1_BASE, Rx, DMA2, DMA2_Stream2, DMA2_Stream2_IRQn, DMA_Channel_4, DMA_IT_TCIF2, DMA_FLAG_TCIF2, DMA_FLAG_HTIF2},
                          {USART3_BASE, Tx, DMA1, DMA1_Stream3, DMA1_Stream3_IRQn, DMA_Channel_4, DMA_IT_TCIF3, DMA_FLAG_TCIF3, DMA_FLAG_HTIF3},
                          {USART3_BASE, Rx, DMA1, DMA1_Stream1, DMA1_Stream1_IRQn, DMA_Channel_4, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                          {USART6_BASE, Tx, DMA2, DMA2_Stream6, DMA2_Stream6_IRQn, DMA_Channel_5, DMA_IT_TCIF6, DMA_FLAG_TCIF6, DMA_FLAG_HTIF6},
                          {USART6_BASE, Rx, DMA2, DMA2_Stream1, DMA2_Stream1_IRQn, DMA_Channel_5, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                          {UART7_BASE, Tx, DMA1, DMA1_Stream1, DMA1_Stream1_IRQn, DMA_Channel_5, DMA_IT_TCIF1, DMA_FLAG_TCIF1, DMA_FLAG_HTIF1},
                          {UART7_BASE, Rx, DMA1, DMA1_Stream3, DMA1_Stream3_IRQn, DMA_Channel_5, DMA_IT_TCIF3, DMA_FLAG_TCIF3, DMA_FLAG_HTIF3},
                          {UART8_BASE, Tx, DMA1, DMA1_Stream0, DMA1_Stream0_IRQn, DMA_Channel_5, DMA_IT_TCIF0, DMA_FLAG_TCIF0, DMA_FLAG_HTIF0},
                          {UART8_BASE, Rx, DMA1, DMA1_Stream6, DMA1_Stream6_IRQn, DMA_Channel_5, DMA_IT_TCIF6, DMA_FLAG_TCIF6, DMA_FLAG_HTIF6},
                          {ADC1_BASE, Rx, DMA2, DMA2_Stream0, DMA2_Stream0_IRQn, DMA_Channel_0, DMA_IT_TCIF0, DMA_FLAG_TCIF0, DMA_FLAG_HTIF0}};

void BSP_DMA_Init(dma_table_index_e tableIndex, uint32_t sourceMemoryAddress, uint32_t bufferSize) {
    // DMA
    DMA_Type dma;
    dma = DMA_Table[tableIndex];
    if (tableIndex <= UART8_Rx) {
        if (dma.TRx == Tx) {
            USART_DMACmd((USART_TypeDef *) dma.PERIPHx_BASE, USART_DMAReq_Tx, ENABLE);
        } else {
            USART_DMACmd((USART_TypeDef *) dma.PERIPHx_BASE, USART_DMAReq_Rx, ENABLE);
        }
    }
    if (tableIndex >= ADC1_Rx && tableIndex <= ADC1_Rx) {
        ADC_DMACmd((ADC_TypeDef *) dma.PERIPHx_BASE, ENABLE);
    }

    DMA_InitTypeDef DMA_InitStructure;
    if (dma.DMAx == DMA1) {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
    } else {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    }
    if (tableIndex <= UART8_Rx) {
        DMA_InitStructure.DMA_PeripheralBaseAddr = &((USART_TypeDef *) dma.PERIPHx_BASE)->DR;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    }
    if (tableIndex >= ADC1_Rx && tableIndex <= ADC1_Rx) {
        DMA_InitStructure.DMA_PeripheralBaseAddr = &((ADC_TypeDef *) dma.PERIPHx_BASE)->DR;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    }

    if (dma.TRx == Tx) {
        DMA_InitStructure.DMA_DIR           = DMA_DIR_MemoryToPeripheral;
        DMA_InitStructure.DMA_FIFOMode      = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    }
    if (dma.TRx == Rx) {
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        if (tableIndex <= UART8_Rx) {
            DMA_InitStructure.DMA_FIFOMode      = DMA_FIFOMode_Enable;
            DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOStatus_HalfFull;
        }
        if (tableIndex >= ADC1_Rx && tableIndex <= ADC1_Rx) {
            DMA_InitStructure.DMA_FIFOMode      = DMA_FIFOMode_Disable;
            DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
        }
    }
    DMA_InitStructure.DMA_Channel         = dma.DMA_Channel_x;
    DMA_InitStructure.DMA_Memory0BaseAddr = sourceMemoryAddress;
    DMA_InitStructure.DMA_BufferSize      = bufferSize;
    DMA_InitStructure.DMA_PeripheralInc   = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc       = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_Priority        = DMA_Priority_Medium;
    DMA_InitStructure.DMA_MemoryBurst     = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(dma.DMAx_Streamy, &DMA_InitStructure);
    DMA_Cmd(dma.DMAx_Streamy, ENABLE);
    // // NVIC
    // NVIC_InitTypeDef NVIC_InitStructure;
    // NVIC_InitStructure.NVIC_IRQChannel                   = dma.DMAx_Streamy_IRQn;
    // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    // NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    // NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    // NVIC_Init(&NVIC_InitStructure);
}

void DMA_Disable(dma_table_index_e tableIndex) {
    DMA_Type dma;
    dma = DMA_Table[tableIndex];
    DMA_Cmd(dma.DMAx_Streamy, DISABLE);
    while (DMA_GetFlagStatus(dma.DMAx_Streamy, dma.DMA_IT_TCIFx) != SET) {
    }
}

void DMA_Enable(dma_table_index_e tableIndex, uint16_t length) {
    DMA_Type dma;
    dma = DMA_Table[tableIndex];
    DMA_ClearFlag(dma.DMAx_Streamy, dma.DMA_FLAG_TCIFx | dma.DMA_FLAG_HTIFx);
    while (DMA_GetCmdStatus(dma.DMAx_Streamy) != DISABLE) {
    }
    DMA_SetCurrDataCounter(dma.DMAx_Streamy, length);
    DMA_Cmd(dma.DMAx_Streamy, ENABLE);
}

DMA_Stream_TypeDef *DMA_Get_Stream(dma_table_index_e tableIndex) {
    DMA_Type dma;
    dma = DMA_Table[tableIndex];
    return dma.DMAx_Streamy;
}

uint16_t DMA_Get_Data_Counter(dma_table_index_e tableIndex) {
    return DMA_GetCurrDataCounter(DMA_Get_Stream(tableIndex));
}

/**
 * @brief 设置PWM所使用的端口
 * @param PWMx   PWM结构体
 * @param PWM_Px 使用的端口,如PWM_PD12
 */
void BSP_PWM_Set_Port(PWM_Type *PWMx, uint32_t PWM_Px) {
    PWMx->RCC_APBxPeriph_TIMx  = PWM_Px >> 28;
    PWMx->TIMx_BASE            = PERIPH_BASE + ((PWM_Px >> 24 & 0x0F) << 16) + ((PWM_Px >> 20 & 0x0F) << 8);
    PWMx->GPIO_AF_TIMx         = PWM_Px >> 16 & 0xF;
    PWMx->Channel              = PWM_Px >> 12 & 0xF;
    PWMx->GPIOx_BASE           = AHB1PERIPH_BASE + ((PWM_Px >> 4 & 0xFF) << 8);
    PWMx->GPIO_PinSourcex      = PWM_Px & 0xF;
    PWMx->RCC_AHB1Periph_GPIOx = 1 << ((PWM_Px >> 4 & 0xFF) / 4);
    PWMx->GPIO_Pin_x           = 1 << (PWM_Px & 0x0F);
    PWMx->CCRx                 = PWMx->Channel * 4 + 0x30;
    PWMx->GPIOx                = (GPIO_TypeDef *) PWMx->GPIOx_BASE;
    PWMx->TIMx                 = (TIM_TypeDef *) PWMx->TIMx_BASE;
}

/**
 * @brief 初始化PWM
 * @note  PI5,PI6,PI7,PI2对应时钟频率为180MHz,其余为90MHz
 * @param PWMx      PWM结构体
 * @param prescaler 预分频器. PWM频率   = TIM/prescaler/period
 * @param period    计数上限. PWM占空比 = compare/period
 * @param polarity  输出极性. TIM_OCPolarity_Low/TIM_OCPolarity_High
 */
void BSP_PWM_Init(PWM_Type *PWMx, uint16_t prescaler, uint32_t period, uint16_t polarity) {
    // InitStructure
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure; // TIM 频率
    GPIO_InitTypeDef        GPIO_InitStructure;        // TIM4_PWM GPIO口设置
    TIM_OCInitTypeDef       TIM_OCInitStructure;       // TIM4_PWM PWM模式

    // GPIO
    RCC_AHB1PeriphClockCmd(PWMx->RCC_AHB1Periph_GPIOx, ENABLE);               // 使能时钟
    GPIO_PinAFConfig(PWMx->GPIOx, PWMx->GPIO_PinSourcex, PWMx->GPIO_AF_TIMx); // GPIO复用为定时器
    GPIO_InitStructure.GPIO_Pin   = PWMx->GPIO_Pin_x;                         // GPIO
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;                             // 复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;                        // 速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                            // 推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;                             // 上拉
    GPIO_Init(PWMx->GPIOx, &GPIO_InitStructure);                              // 初始化

    // TIM
    if (PWMx->TIMx == TIM8) {
        RCC_APB2PeriphClockCmd(PWMx->RCC_APBxPeriph_TIMx, ENABLE); // 时钟使能
    } else {
        RCC_APB1PeriphClockCmd(PWMx->RCC_APBxPeriph_TIMx, ENABLE); // 时钟使能
    }
    TIM_TimeBaseInitStructure.TIM_Prescaler     = prescaler - 1;      // 定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_Period        = period - 1;         // 自动重装载值
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;       // ClockDivision
    TIM_TimeBaseInit(PWMx->TIMx, &TIM_TimeBaseInitStructure);         // 初始化定时器

    // TIM_OC
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM2;        // 选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = polarity;               // 输出极性:TIM输出比较极性低
    TIM_OCInitStructure.TIM_Pulse       = 5;                      // Pulse
    if (PWMx->Channel == 1) {
        TIM_OC1Init(PWMx->TIMx, &TIM_OCInitStructure);          // 根据指定的参数初始化外设
        TIM_OC1PreloadConfig(PWMx->TIMx, TIM_OCPreload_Enable); // 使能TIM在CCR1上的预装载寄存器
    } else if (PWMx->Channel == 2) {
        TIM_OC2Init(PWMx->TIMx, &TIM_OCInitStructure);          // 根据指定的参数初始化外设
        TIM_OC2PreloadConfig(PWMx->TIMx, TIM_OCPreload_Enable); // 使能TIM在CCR1上的预装载寄存器
    } else if (PWMx->Channel == 3) {
        TIM_OC3Init(PWMx->TIMx, &TIM_OCInitStructure);          // 根据指定的参数初始化外设
        TIM_OC3PreloadConfig(PWMx->TIMx, TIM_OCPreload_Enable); // 使能TIM在CCR1上的预装载寄存器
    } else if (PWMx->Channel == 4) {
        TIM_OC4Init(PWMx->TIMx, &TIM_OCInitStructure);          // 根据指定的参数初始化外设
        TIM_OC4PreloadConfig(PWMx->TIMx, TIM_OCPreload_Enable); // 使能TIM在CCR1上的预装载寄存器
    }
    TIM_ARRPreloadConfig(PWMx->TIMx, ENABLE); // ARPE使能
    TIM_Cmd(PWMx->TIMx, ENABLE);              // 使能TIM

    // 高级定时器需要使能MOE位
    if (PWMx->TIMx == TIM1 || PWMx->TIMx == TIM8) {
        TIM_CtrlPWMOutputs(PWMx->TIMx, ENABLE);
    }
}

/**
 * @brief 设置占空比
 * @param PWMx    PWM结构体
 * @param compare 比较值. PWM占空比 = compare/period
 */
void PWM_Set_Compare(PWM_Type *PWMx, uint32_t compare) {
    *((uint32_t *) (((uint8_t *) PWMx->TIMx) + PWMx->CCRx)) = compare;
}

void BSP_LED_Init(void) {
    // 用户自定义LED*8
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = 0x01FE;            // GPIO_Pin_1-GPIO_Pin_8
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;     // 普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     // 推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;      // 上拉
    GPIO_Init(GPIOG, &GPIO_InitStructure);             // 初始化
    GPIO_SetBits(GPIOG, 0x01FE);                       // GPIOG1-8设置高,灯灭
}

/**
 * @brief 设置LED亮暗
 * @param row 共8位表示8个灯,1为亮,0为暗
 */
void LED_Set_Row(uint16_t row) {
    GPIO_SetBits(GPIOG, (~(row << 1)) & 0x1FE);
    GPIO_ResetBits(GPIOG, (row << 1));
}

/**
 * @brief 设置LED亮起数量
 * @param progress 0-8
 */
void LED_Set_Progress(uint16_t progress) {
    // 设置
    MIAO(progress, 0, 8);
    LED_Set_Row((1 << progress) - 1);
}

/**
 * @brief 炫酷跑马灯
 * @note  每次调用本函数会更新LED状态,但没有延时
 *        建议每次调用后设置20ms延时
 */
void LED_Run_Horse() {
    static uint16_t LEDHorseRow   = 0;
    static uint16_t LEDHorseState = 0;
    LEDHorseState                 = (LEDHorseState % 26) + 1;
    if (LEDHorseState <= 8)
        LEDHorseRow = (LEDHorseRow << 1) + 1;
    else if (LEDHorseState >= 14 && LEDHorseState <= 21)
        LEDHorseRow = LEDHorseRow - (1 << LEDHorseState - 14);
    LED_Set_Row(LEDHorseRow);
}

/**
 * @brief Windows XP开机动画跑马灯
 * @note  每次调用本函数会更新LED状态,但没有延时
 *        建议每次调用后设置200ms延时
 */
void LED_Run_Horse_XP() {
    static uint16_t LEDXPRow   = 0;
    static uint16_t LEDXPState = 0;
    LEDXPState                 = (LEDXPState % 11) + 1;
    if (LEDXPState <= 3)
        LEDXPRow = (LEDXPRow << 1) + 1;
    else if (LEDXPState >= 4 && LEDXPState <= 8)
        LEDXPRow = LEDXPRow << 1;
    else
        LEDXPRow = LEDXPRow - (1 << LEDXPState - 4);
    LED_Set_Row(LEDXPRow);
}

void BSP_Beep_Init(void) {
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE); // TIM12时钟使能
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); //使能GPIOH时钟

    GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_TIM12); // GPIOH6复用为定时器12

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;        // GPIOH6
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;      //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;     //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;      //上拉
    GPIO_Init(GPIOH, &GPIO_InitStructure);             //初始化PH6

    TIM_TimeBaseStructure.TIM_Prescaler     = 90 - 1;             //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_Period        = 1;                  //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure); //初始化定时器3

    //初始化TIM12 Channel1 PWM模式
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;        //选择定时器模式:TIM脉冲宽度调制模式2
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;    //输出极性:TIM输出比较极性高
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);                     //根据T指定的参数初始化外设TIM12 OC1

    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable); //使能TIM12在CCR1上的预装载寄存器

    TIM_ARRPreloadConfig(TIM12, ENABLE); // ARPE使能

    TIM_Cmd(TIM12, ENABLE); //使能TIM3
}

extern Sound_Tone_Type Music_Scope_XP[];
extern Sound_Tone_Type Music_Scope_Earth[];
extern Sound_Tone_Type Music_Scope_Sky[];
extern Sound_Tone_Type Music_Scope_Soul[];
/**
 * @brief 音乐点播~
 * @note  1. 如果播放完成返回1,任务应退出循环并自杀
 *        2. 谱子在beep.c里
 *
 * @song  名称            歌曲        推荐延时(ms)
 *        Music_Sky       天空之城    350
 *        Music_Earth     极乐净土    120
 *        Music_Soul      New Soul   60
 *        Music_XP        XP开机音乐  150
 */
uint8_t KTV_Play(Song_Type song) {
    static uint32_t index = 0;
    switch (song) {
    case Music_Sky:
        if (index >= Music_Len_Sky) return 1;
        Sing(Music_Scope_Sky[index]);
        break;
    case Music_Earth:
        if (index >= Music_Len_Earth) return 1;
        Sing(Music_Scope_Earth[index]);
        break;
    case Music_Soul:
        if (index >= Music_Len_Soul) return 1;
        Sing(Music_Scope_Soul[index]);
        break;
    case Music_XP:
        if (index >= Music_Len_XP) return 1;
        Sing(Music_Scope_XP[index]);
        break;
    default:
        break;
    }
    index++;
    return 0;
}

void BSP_I2C2_Init(void) {
    //占坑
}

void BSP_Button_Init(void) {
    //占坑
}
