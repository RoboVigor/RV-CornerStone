/**
 * @brief 硬件初始化
 * @note 该函数应该在task.c中被执行
 */

#include "bsp.h"
#include "Driver_BSP.h"
#include "handle.h"

void BSP_Init(void) {
  BSP_CAN_Init();
  BSP_DBUS_Init(remoteBuffer);
  BSP_TIM2_Init();
  BSP_IMU_Init();
  BSP_Laser_Init();
  BSP_Beep_Init();
  BSP_LED_Init();
  BSP_User_Power_Init();

  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //   GPIO_ResetetBits(GPIOD, GPIO_Pin_14);

  // snail 电机pwm输出
  BSP_PWM_Set_Port(&PWM_Snail1, PWM_PORT_PD12);
  BSP_PWM_Init(&PWM_Snail1, 180, 1250, TIM_OCPolarity_Low);
  BSP_PWM_Set_Port(&PWM_Snail2, PWM_PORT_PD13);
  BSP_PWM_Init(&PWM_Snail2, 180, 1250, TIM_OCPolarity_Low);

  // USART
  // BSP_USART2_Init(9600, USART_IT_RXNE);

  // Judge (USART6)
  // BSP_USART6_Init(115200, 0);
  // BSP_DMA_USART6_RX_Init(Judge.buf, Protocol_Buffer_Length);

  // Ps (USART3)
  // BSP_USART3_Init(115200, 0);
  // BSP_DMA_USART3_RX_Init(Ps.buf, Protocol_Buffer_Length);

  // PWM
  // BSP_PWM_Set_Port(&PWM_Test, PWM_PORT_PD12);
  // BSP_PWM_Init(&PWM_Test, 9000, 200, TIM_OCPolarity_Low);
}
