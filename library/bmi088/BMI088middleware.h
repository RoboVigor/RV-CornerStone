#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "stm32f4xx.h"

#define CS1_ACCEL_Pin GPIO_Pin_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define CS1_GYRO_Pin GPIO_Pin_0
#define CS1_GYRO_GPIO_Port GPIOB
#define INT1_ACCEL_Pin GPIO_Pin_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
#define INT1_GRYO_Pin GPIO_Pin_5
#define INT1_GRYO_GPIO_Port GPIOC
#define INT1_GRYO_EXTI_IRQn EXTI9_5_IRQn

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#endif
