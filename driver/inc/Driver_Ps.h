/**
 * @file Driver_Ps.h
 * @brief 视觉辅助
 */

#ifndef __DRIVER_PS_H
#define __DRIVER_PS_H

#include "stm32f4xx.h"

#define SOP (uint8_t)87 // start of package
#define EOP (uint8_t)88 // end of package

typedef struct {
  uint8_t data[34];
  uint16_t result[10];
  uint32_t id;
} PsData_Type;

void Ps_Append(PsData_Type *PsData, uint8_t value);
void Ps_Parse_Header(PsData_Type *PsData, uint8_t value);
void Ps_Valid(PsData_Type *PsData);
void Ps_Reset(PsData_Type *PsData);

void Ps_On_Start(PsData_Type *PsData);
void Ps_On_Done(PsData_Type *PsData);
void Ps_On_Interrupted(PsData_Type *PsData);
void Ps_On_Received(PsData_Type *PsData, uint8_t newByte);

void Ps_DataAnalysis(PsData_Type *PsData);
void Ps_DataUnused(PsData_Type *PsData);
void Ps_DataUsed(PsData_Type *PsData);

uint8_t CRC_Calculate(uint8_t *data, uint8_t len);
uint8_t CRC_Valid(uint8_t *data, uint8_t len, uint8_t crc);

#endif
