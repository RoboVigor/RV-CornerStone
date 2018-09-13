#include "Driver_DBUS.h"

#include "handle.h"

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

    DBusData.ch1 = (DBusBuffer[0] | DBusBuffer[1] << 8) & 0x07FF;
    DBusData.ch1 -= 1024;
    DBusData.ch2 = (DBusBuffer[1] >> 3 | DBusBuffer[2] << 5) & 0x07FF;
    DBusData.ch2 -= 1024;
    DBusData.ch3 = (DBusBuffer[2] >> 6 | DBusBuffer[3] << 2 | DBusBuffer[4] << 10) & 0x07FF;
    DBusData.ch3 -= 1024;
    DBusData.ch4 = (DBusBuffer[4] >> 1 | DBusBuffer[5] << 7) & 0x07FF;
    DBusData.ch4 -= 1024;

    DBusData.switchLeft  = ((DBusBuffer[5] >> 4) & 0x000C) >> 2;
    DBusData.switchRight = (DBusBuffer[5] >> 4) & 0x0003;

    DBusData.mouse.x = DBusBuffer[6] | (DBusBuffer[7] << 8); // x axis
    DBusData.mouse.y = DBusBuffer[8] | (DBusBuffer[9] << 8);
    DBusData.mouse.z = DBusBuffer[10] | (DBusBuffer[11] << 8);

    DBusData.mouse.pressLeft  = DBusBuffer[12]; // is pressed?
    DBusData.mouse.pressRight = DBusBuffer[13];

    DBusData.keyBoard.keyCode = DBusBuffer[14] | DBusBuffer[15] << 8; // key borad code
}
