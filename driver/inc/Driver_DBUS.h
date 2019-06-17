/**
 * @file Driver_DBus.h
 * @brief 遥控器驱动
 */

#ifndef __DRIVER_DBUS_H
#define __DRIVER_DBUS_H

#include "stm32f4xx.h"

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

    struct {
        int16_t x;
        int16_t y;
        int16_t z;

        uint8_t pressLeft;
        uint8_t pressRight;

        uint8_t jumpPressLeft;
        uint8_t jumpPressRight;
    } mouse;

    struct {
        /**********************************************************************************
         * 键盘通道:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
         *          V    C    X	   Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
         ************************************************************************************/
        uint16_t keyCode;     //原始键值
        uint16_t jumpKeyCode; //跳变后的键值
    } keyBoard;
} DBusData_Type;

/**
 * @brief DBUS解码
 *
 * @param DBusData
 */
void DBus_Update(DBusData_Type *DBusData, uint8_t DBusBuffer[]);

#endif
