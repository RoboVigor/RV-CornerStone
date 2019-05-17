/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "handle.h"
#include "Driver_BSP.h"

void BSP_Init(void) {
    BSP_CAN_Init();
    BSP_DBUS_Init(remoteBuffer);
    BSP_TIM2_Init();
    BSP_IMU_Init();
    BSP_USART3_Init(9600);
    BSP_USART6_Init(9600);
    BSP_UART7_Init(9600);
    BSP_UART8_Init(9600);
    BSP_Laser_Init();
    BSP_PWM_Init(PWM_PD12, 9000, 200);
}
