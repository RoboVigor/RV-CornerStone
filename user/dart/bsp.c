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
    BSP_IMU_Init();
    BSP_Laser_Init();
    BSP_Beep_Init();
    BSP_LED_Init();

#ifdef STM32F427_437xx
    BSP_User_Power_Init();
#endif

#ifdef STM32F407xx
    // // USART1
    // BSP_USART1_Init(115200, USART_IT_IDLE);
    // BSP_DMA_Init(USART1_Tx, UserChannel.sendBuf, Protocol_Buffer_Length);
    // BSP_DMA_Init(USART1_Rx, UserChannel.receiveBuf, Protocol_Buffer_Length);
#endif

    // PWM
    BSP_PWM_Set_Port(&PWM_Test, PWM_PI7);                    // 180MHz
    BSP_PWM_Init(&PWM_Test, 36000, 100, TIM_OCPolarity_Low); // 50Hz
}
