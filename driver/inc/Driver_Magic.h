/**
 * @file Driver_Magic.h
 * @brief 调试相关函数
 * @note  调试函数不遵守"命名空间"的规范
 */
#ifndef __DRIVER_MAGIC_H
#define __DRIVER_MAGIC_H

#include "stm32f4xx_conf.h"
#include "Driver_Protocol.h"
#include "Driver_Bridge.h"
#include "protocol.h"

/**
 * @brief 发送报错信息
 * @todo  未测试
 */
void Raise_Error(Node_Type *node, uint16_t code, char *text);

/**
 * @brief 发送调试信息
 * @todo  未测试
 */
void Send_Debug_Info(Node_Type *node, DebugData_Type debugData);

/**
 * @brief 陀螺仪校准
 * @todo  未实现
 */
void Clibrate_Gyroscope();

/**
 * @brief 陀螺仪校准
 * @todo  未实现
 */
void Inspect_RTOS();

#endif
