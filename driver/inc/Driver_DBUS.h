/**
 * @file Driver_DBus.h
 * @brief 遥控器驱动
 */

#ifndef __DRIVER_DBUS_H
#define __DRIVER_DBUS_H

#include "stm32f4xx.h"
#include "vegmath.h"

// DBUS接收数据帧长度
#define DBUS_LENGTH 18     // DBUS数据帧长
#define DBUS_BACK_LENGTH 1 //增加一个字节保持稳定

//所有按键对应位
#define KEY_V 0x4000
#define KEY_C 0x2000
#define KEY_X 0x1000
#define KEY_Z 0x0800
#define KEY_G 0x0400
#define KEY_F 0x0200
#define KEY_R 0x0100
#define KEY_E 0x0080
#define KEY_Q 0x0040
#define KEY_CTRL 0x0020
#define KEY_SHIFT 0x0010
#define KEY_D 0x0008
#define KEY_A 0x0004
#define KEY_S 0x0002
#define KEY_W 0x0001

enum DBusState { DBusIdle, DBusWorking };

//遥控解码数据存储结构体
typedef struct {
    union {
        struct {
            int16_t rx, ry, lx, ly;
        };
        struct {
            int16_t ch1, ch2, ch3, ch4;
        };
    };

    enum DBusState state;

    uint8_t switchLeft; // 3 value
    uint8_t switchRight;
} Remote_Type;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t pressLeft;
    uint8_t pressRight;
} Mouse_Type;

/**********************************************************************************
 * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
 *          V    C    X	   Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
 ************************************************************************************/

typedef struct {
    union {
        struct {
            unsigned int W : 1;
            unsigned int S : 1;
            unsigned int A : 1;
            unsigned int D : 1;
            unsigned int Shift : 1;
            unsigned int Ctrl : 1;
            unsigned int Q : 1;
            unsigned int E : 1;
            unsigned int R : 1;
            unsigned int F : 1;
            unsigned int G : 1;
            unsigned int Z : 1;
            unsigned int X : 1;
            unsigned int C : 1;
            unsigned int V : 1;
        };
        struct {
            uint16_t keyCode;
        };
    };
    uint16_t keyDisabledCounter[15];
    uint16_t keyDisabledCode;
    uint16_t seq;

    enum DBusState state;
} Keyboard_Type;

/**
 * @brief DBUS解码
 *
 * @param DBusData
 */

void DBus_Update(Remote_Type *remote, Keyboard_Type *kb, Mouse_Type *mouse, uint8_t DBusBuffer[]);

/**
 * @brief 暂时禁用某键
 *
 * @param kb
 * @param key 宏定义KEY_X
 * @param duration 禁用时间 (ms)
 */
void Key_Disable(Keyboard_Type *kb, uint16_t key, uint16_t duration);

#endif
