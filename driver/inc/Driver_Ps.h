/**
 * @file Driver_Ps.h
 * @brief 视觉辅助
 */

#ifndef __DRIVER_PS_H
#define __DRIVER_PS_H

#include "stm32f4xx.h"

#define SOP (uint8_t) 87 // start of package
#define EOP (uint8_t) 88 // end of package

void Ps_Append(uint8_t value);
void Ps_Parse_Header(uint8_t value);
void Ps_Valid(void);
void Ps_Reset(void);

void Ps_On_Start(void);
void Ps_On_Done(void);
void Ps_On_Interrupted(void);
void Ps_On_Received(uint8_t newByte);

void Ps_DataAnalysis(void);
void Ps_DataUnused(void);
void Ps_DataUsed(void);

uint8_t CRC_Calculate(uint8_t *data, uint8_t len);
uint8_t CRC_Valid(uint8_t *data, uint8_t len, uint8_t crc);

#endif
