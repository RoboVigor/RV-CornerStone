#define __DBUS_GLOBALS

#include "Driver_DBUS.h"

/**
 * @brief  DBUS参数初始化
 * @param  void
 * @return void
 */
void Dbus_Init(void) {
    DbusData.ch1                  = 0;
    DbusData.ch2                  = 0;
    DbusData.ch3                  = 0;
    DbusData.ch4                  = 0;
    DbusData.keyBoard.keyCode     = 0;
    DbusData.keyBoard.jumpKeyCode = 0;
}

/**
 * @brief  DBUS解码
 * @param  void
 * @return void
 */
void Dbus_Decode_Remote_Control_Data(void) {
    OldDbusData = DbusData;

    DbusData.ch1 = (dbusBuffer[0] | dbusBuffer[1] << 8) & 0x07FF;
    DbusData.ch1 -= 1024;
    DbusData.ch2 = (dbusBuffer[1] >> 3 | dbusBuffer[2] << 5) & 0x07FF;
    DbusData.ch2 -= 1024;
    DbusData.ch3 = (dbusBuffer[2] >> 6 | dbusBuffer[3] << 2 | dbusBuffer[4] << 10) & 0x07FF;
    DbusData.ch3 -= 1024;
    DbusData.ch4 = (dbusBuffer[4] >> 1 | dbusBuffer[5] << 7) & 0x07FF;
    DbusData.ch4 -= 1024;

    DbusData.switchLeft  = ((dbusBuffer[5] >> 4) & 0x000C) >> 2;
    DbusData.switchRight = (dbusBuffer[5] >> 4) & 0x0003;

    DbusData.mouse.x = dbusBuffer[6] | (dbusBuffer[7] << 8); // x axis
    DbusData.mouse.y = dbusBuffer[8] | (dbusBuffer[9] << 8);
    DbusData.mouse.z = dbusBuffer[10] | (dbusBuffer[11] << 8);

    DbusData.mouse.pressLeft  = dbusBuffer[12]; // is pressed?
    DbusData.mouse.pressRight = dbusBuffer[13];

    DbusData.keyBoard.keyCode = dbusBuffer[14] | dbusBuffer[15] << 8; // key borad code
}
