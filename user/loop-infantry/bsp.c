/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "handle.h"

void BSP_Capacitor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // Discharge
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Charge
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void BSP_ADC_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    // Discharge
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Charge
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void BSP_ADC_DMA_Init(void) {

    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA2_Channel0);                                                  //将DMA的通道1寄存器重设为缺省值
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) &ADC1->DR;                 // DMA外设ADC基地址
    DMA_InitStructure.DMA_MemoryBaseAddr     = (u32) &ADC_Value;                // DMA内存基地址
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;           //内存作为数据传输的目的地
    DMA_InitStructure.DMA_BufferSize         = 40;                              // DMA通道的DMA缓存的大小
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //外设地址寄存器不变
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //内存地址寄存器递增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;     //数据宽度为16位
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;               //工作在循环缓存模式
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;               // DMA通道 x拥有高优先级
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;                 // DMA通道x没有设置为内存到内存传输
    DMA_Init(DMA2_Channel0, &DMA_InitStructure);                                //根据DMA_InitStruct中指定的参数初始化DMA的通道
}
void BSP_Init(void) {
    BSP_CAN_Init();
    BSP_DBUS_Init(remoteBuffer);
    BSP_TIM2_Init();
    BSP_IMU_Init();
    BSP_Capacitor_Init();
    BSP_ADC_Init();
    BSP_ADC_GPIO_Init();
    BSP_ADC_DMA_Init();
    BSP_Laser_Init();
    BSP_Beep_Init();
    BSP_LED_Init();
    BSP_User_Power_Init();

    // USART
    // BSP_USART2_Init(9600, USART_IT_RXNE);

#ifdef ROBOT_LOOP_ONE
    // Ps (UART8)
    BSP_UART8_Init(115200, USART_IT_IDLE);
    BSP_DMA_UART8_RX_Init(Ps.receiveBuf, Protocol_Buffer_Length);
    BSP_DMA_UART8_TX_Init(Ps.sendBuf, Protocol_Buffer_Length);
#endif

#ifdef ROBOT_LOOP_TWO
    // Ps (USART3)
    BSP_USART3_Init(115200, USART_IT_IDLE);
    BSP_DMA_USART3_RX_Init(Ps.receiveBuf, Protocol_Buffer_Length);
    BSP_DMA_USART3_TX_Init(Ps.sendBuf, Protocol_Buffer_Length);
#endif

#ifdef ROBOT_LOOP_THREE
    // Ps (USART3)
    BSP_USART3_Init(115200, USART_IT_IDLE);
    BSP_DMA_USART3_RX_Init(Ps.receiveBuf, Protocol_Buffer_Length);
    BSP_DMA_USART3_TX_Init(Ps.sendBuf, Protocol_Buffer_Length);
#endif

    // Judge (USART6)
    BSP_USART6_Init(115200, USART_IT_IDLE);
    BSP_DMA_USART6_RX_Init(Judge.receiveBuf, Protocol_Buffer_Length);
    BSP_DMA_USART6_TX_Init(Judge.sendBuf, Protocol_Buffer_Length);

    // Servo
    BSP_PWM_Set_Port(&PWM_Magazine_Servo, PWM_PORT_PI0);
    BSP_PWM_Init(&PWM_Magazine_Servo, 9000, 200, TIM_OCPolarity_Low);

    // snail 电机pwm输出
    BSP_PWM_Set_Port(&PWM_Snail1, PWM_PORT_PD12);
    BSP_PWM_Init(&PWM_Snail1, 180, 1250, TIM_OCPolarity_Low);
    BSP_PWM_Set_Port(&PWM_Snail2, PWM_PORT_PD13);
    BSP_PWM_Init(&PWM_Snail2, 180, 1250, TIM_OCPolarity_Low);
}
