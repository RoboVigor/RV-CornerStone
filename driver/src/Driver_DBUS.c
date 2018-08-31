#define __DBUS_GLOBALS

#include "Driver_DBUS.h"

/**
 * @brief  DBUS参数初始化
 * @param  void
 * @return void
 */
void DBus_Init(void) {
    DBusData.ch1                  = 0;
    DBusData.ch2                  = 0;
    DBusData.ch3                  = 0;
    DBusData.ch4                  = 0;
    DBusData.keyBoard.keyCode     = 0;
    DBusData.keyBoard.jumpKeyCode = 0;
}

/**
 * @brief  DBUS解码
 * @param  void
 * @return void
 */
void DBus_Decode_Remote_Control_Data(void) {
    LastDBusData = DBusData;

    DBusData.ch1 = (dBusBuffer[0] | dBusBuffer[1] << 8) & 0x07FF;
    DBusData.ch1 -= 1024;
    DBusData.ch2 = (dBusBuffer[1] >> 3 | dBusBuffer[2] << 5) & 0x07FF;
    DBusData.ch2 -= 1024;
    DBusData.ch3 = (dBusBuffer[2] >> 6 | dBusBuffer[3] << 2 | dBusBuffer[4] << 10) & 0x07FF;
    DBusData.ch3 -= 1024;
    DBusData.ch4 = (dBusBuffer[4] >> 1 | dBusBuffer[5] << 7) & 0x07FF;
    DBusData.ch4 -= 1024;

    DBusData.switchLeft  = ((dBusBuffer[5] >> 4) & 0x000C) >> 2;
    DBusData.switchRight = (dBusBuffer[5] >> 4) & 0x0003;

    DBusData.mouse.x = dBusBuffer[6] | (dBusBuffer[7] << 8); // x axis
    DBusData.mouse.y = dBusBuffer[8] | (dBusBuffer[9] << 8);
    DBusData.mouse.z = dBusBuffer[10] | (dBusBuffer[11] << 8);

    DBusData.mouse.pressLeft  = dBusBuffer[12]; // is pressed?
    DBusData.mouse.pressRight = dBusBuffer[13];

    DBusData.keyBoard.keyCode = dBusBuffer[14] | dBusBuffer[15] << 8; // key borad code
}
