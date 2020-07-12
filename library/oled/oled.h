/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.h
 * @brief      this file contains sd card basic operating function
 * @note
 * @Version    V1.0.0
 * @Date       Jan-28-2018
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_spi.h"
#include <stdint.h>

#ifdef STM32F427_437xx
#define OLED_USE_SPI
#define Max_Column 130
#endif
#ifdef STM32F407xx
#define OLED_USE_IIC
#define OLED_I2C_ADDRESS 0x3c
#define Max_Column 128
#endif

#define Max_Row 64

#define X_WIDTH Max_Column
#define Y_WIDTH Max_Row

#define OLED_CMD 0x00
#define OLED_DATA 0x01

#define CHAR_SIZE_WIDTH 6
#define VHAR_SIZE_HIGHT 12

#define OLED_DC_GPIO_Port GPIOB
#define OLED_RST_GPIO_Port GPIOB

#define OLED_DC_Pin GPIO_Pin_9
#define OLED_RST_Pin GPIO_Pin_10

#define RELEASE_LIMIT 4096
#define DOWN_LIMIT 3400
#define UP_LIMIT 2550
#define RIGHT_LIMIT 1800
#define LEFT_LIMIT 900
#define CLICK_LIMIT 10

#define OLED_CMD_Set() GPIO_SetBits(OLED_DC_GPIO_Port, OLED_DC_Pin)
#define OLED_CMD_Clr() GPIO_ResetBits(OLED_DC_GPIO_Port, OLED_DC_Pin)

#define OLED_RST_Set() GPIO_SetBits(OLED_RST_GPIO_Port, OLED_RST_Pin)
#define OLED_RST_Clr() GPIO_ResetBits(OLED_RST_GPIO_Port, OLED_RST_Pin)

/* type define */
typedef void (*MenuFunction)();

/* struct define */
typedef struct {
    char         title[20];
    void *       next;
    MenuFunction func;
} MenuItem;

/* enum define */
typedef enum {
    Pen_Clear     = 0x00,
    Pen_Write     = 0x01,
    Pen_Inversion = 0x02,
} Pen_Typedef;

/* function define */
void oled_init(void);
void oled_write_byte(uint8_t dat, uint8_t cmd);
void oled_display_on(void);
void oled_display_off(void);
void oled_refresh_gram(void);
void oled_clear(Pen_Typedef pen);
void oled_drawpoint(int8_t x, int8_t y, Pen_Typedef pen);
void oled_drawline(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, Pen_Typedef pen);
void oled_showchar(uint8_t row, uint8_t col, uint8_t chr);
void oled_shownum(uint8_t row, uint8_t col, uint32_t num, uint8_t mode, uint8_t len);
void oled_showstring(uint8_t row, uint8_t col, uint8_t *chr);
void oled_printf(uint8_t row, uint8_t col, const char *fmt, ...);
void oled_LOGO(void);
void oled_menu(uint16_t joystickValue);

#endif
