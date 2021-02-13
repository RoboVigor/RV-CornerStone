/**
 * @brief 初始化全局变量(结构体)
 *
 */
#include "handle.h"
#include "config.h"

void Handle_Init(void) {
    // 遥控器数据初始化
    DBUS_Init(&remoteData, &keyboardData, &mouseData);
}
