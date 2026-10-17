#include "led_app.h"
#include "stm32f4xx_hal.h"

#define LED_PINS (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | \
                  GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)

static uint16_t current_pin;

void LED_App_Init(void)
{
    HAL_GPIO_WritePin(GPIOG, LED_PINS, GPIO_PIN_SET);
    current_pin = GPIO_PIN_1;
}

void LED_App_Step(void)
{
    HAL_GPIO_WritePin(GPIOG, current_pin, GPIO_PIN_RESET);
    HAL_Delay(1000U);
    HAL_GPIO_WritePin(GPIOG, current_pin, GPIO_PIN_SET);
    current_pin = (current_pin == GPIO_PIN_8)
                    ? GPIO_PIN_1 : (uint16_t)(current_pin << 1U);
}
