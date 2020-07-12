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
    BSP_OLED_Init();
    BSP_Button_Init();

#ifdef STM32F427_437xx
    BSP_User_Power_Init();
#endif

#ifdef STM32F40_41xxx
    // 涵道
    BSP_PWM_Set_Port(&PWM_Motor_Duct, PWM_PE9);                    // 180MHz
    BSP_PWM_Init(&PWM_Motor_Duct, 36000, 100, TIM_OCPolarity_Low); // 50Hz
    PWM_Set_Compare(&PWM_Motor_Duct, 5);

    // Yaw轴舵机
    BSP_PWM_Set_Port(&PWM_Servo_Yaw, PWM_PE11);                   // 180MHz
    BSP_PWM_Init(&PWM_Servo_Yaw, 36000, 100, TIM_OCPolarity_Low); // 50Hz

    // Pitch轴舵机
    BSP_PWM_Set_Port(&PWM_Servo_Pitch, PWM_PE13);                   // 180MHz
    BSP_PWM_Init(&PWM_Servo_Pitch, 36000, 100, TIM_OCPolarity_Low); // 50Hz
#endif
}
