/**
 * @file Driver_Ps.h
 * @brief 视觉通讯
 */

#ifndef __DRIVER_PS_H
#define __DRIVER_PS_H

#include "stm32f4xx.h"

#define SOF (uint8_t) 87 // start of file
#define EOF (uint8_t) 88 // end of file

void Ps_Init_Config(void);

/**
 * @brief 接收字节
 *
 * @param newByte
 */
void Ps_On_Received(uint8_t newByte);

void Ps_DataAnalysis(void);
void Ps_DataUnused(void);
void Ps_DataUsed(void);

#endif
