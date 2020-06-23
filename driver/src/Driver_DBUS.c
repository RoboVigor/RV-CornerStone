#include "Driver_DBUS.h"

void DBUS_Init(Remote_Type *remote, Keyboard_Type *kb, Mouse_Type *mouse) {
    remote->state = DBusIdle;
    kb->state     = DBusIdle;
    kb->seq       = 0;

    remote->ch1 = 0;
    remote->ch2 = 0;
    remote->ch3 = 0;
    remote->ch4 = 0;

    mouse->x = 0;
    mouse->y = 0;
    mouse->z = 0;

    kb->keyCode = 0;
}

void DBus_Update(Remote_Type *remote, Keyboard_Type *kb, Mouse_Type *mouse, uint8_t DBusBuffer[]) {
    int i;
    remote->state = DBusWorking;

    remote->ch1 = (DBusBuffer[0] | DBusBuffer[1] << 8) & 0x07FF;
    remote->ch1 -= 1024;
    remote->ch2 = (DBusBuffer[1] >> 3 | DBusBuffer[2] << 5) & 0x07FF;
    remote->ch2 -= 1024;
    remote->ch3 = (DBusBuffer[2] >> 6 | DBusBuffer[3] << 2 | DBusBuffer[4] << 10) & 0x07FF;
    remote->ch3 -= 1024;
    remote->ch4 = (DBusBuffer[4] >> 1 | DBusBuffer[5] << 7) & 0x07FF;
    remote->ch4 -= 1024;

    remote->switchLeft  = ((DBusBuffer[5] >> 4) & 0x000C) >> 2;
    remote->switchRight = (DBusBuffer[5] >> 4) & 0x0003;

    mouse->x = DBusBuffer[6] | (DBusBuffer[7] << 8);
    mouse->y = DBusBuffer[8] | (DBusBuffer[9] << 8);
    mouse->z = DBusBuffer[10] | (DBusBuffer[11] << 8);

    mouse->pressLeft  = DBusBuffer[12];
    mouse->pressRight = DBusBuffer[13];

    // 按键禁用
    kb->seq             = (kb->seq++) % 1024;
    kb->keyDisabledCode = 0;
    for (i = 0; i < 15; i++) {
        if (kb->keyDisabledCounter[i] > 0) {
            kb->keyDisabledCounter[i]--;
            kb->keyDisabledCode += 1 << i;
        }
    }

    // kb->keyCode = (DBusBuffer[14] | DBusBuffer[15] << 8);
    // kb->keyCode = kb->keyCode & (~kb->keyDisabledCode);
    // kb->keyCode = kb->keyCode;

    kb->keyCode = (DBusBuffer[14] | DBusBuffer[15] << 8) & (~kb->keyDisabledCode);

    if (kb->keyCode != 0 || mouse->x != 0) {
        kb->state = DBusWorking;
    }
}

void Key_Disable(Keyboard_Type *kb, uint16_t key, uint16_t duration) {
    kb->keyDisabledCounter[FastLog2(key)] = duration;
}