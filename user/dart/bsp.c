/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "handle.h"

void BSP_Init(void) {
    BSP_CAN_Init();
    BSP_DBUS_Init(remoteBuffer);
    BSP_TIM2_Init();
    BSP_Beep_Init();
    BSP_LED_Init();

    // // USART3
    // BSP_USART1_Init(115200, USART_IT_IDLE);
    // BSP_DMA_Init(USART1_Tx, UserChannel.sendBuf, Protocol_Buffer_Length);
    // BSP_DMA_Init(USART1_Rx, UserChannel.receiveBuf, Protocol_Buffer_Length);

    // PWM
    // BSP_PWM_Set_Port(&PWM_Test, PWM_PE9);                   // 180MHz
    // BSP_PWM_Init(&PWM_Test, 9000, 200, TIM_OCPolarity_Low); // 100Hz
}
