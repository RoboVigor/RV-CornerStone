/* Syntax-only STM32 adapter boundary, NOT a replacement firmware platform.
 * Uses real vendor/CMSIS declarations, substitutes only the unrelated legacy
 * handle.h graph (which requires Keil ARMCC). Full project compilation remains
 * a required Windows gate. Never include this header in a firmware target. */
#ifndef GIMBAL_PLATFORM_SYNTAX_STUB_H
#define GIMBAL_PLATFORM_SYNTAX_STUB_H
#define __HANDLE_H
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#ifndef GIMBAL_TUNING_ENABLED
#define GIMBAL_TUNING_ENABLED 1
#endif
#define portTICK_PERIOD_MS 1u
uint32_t xTaskGetTickCount(void);
typedef struct {
    float angle, actualCurrent;
    int16_t speed, temperature, input;
    uint16_t updatedAt;
    uint8_t online;
    int8_t inputEnabled, direction;
} Motor_Type;
typedef struct {
    struct { float angle,speed; uint8_t online,error_code; uint32_t updated_at; } state;
} Unitree_Motor_Type;
extern Motor_Type Motor6020_MecArm_0_0,Motor6020_MecArm_0_1,Motor6020_MecArm_3;
extern Unitree_Motor_Type Unitree_MecArm_1,Unitree_MecArm_2;
extern uint8_t SafetyMode;
extern struct { float joint_target[5]; } MecArm;
#endif
