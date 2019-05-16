/**
 * @file Driver_Ps.h
 * @brief 视觉辅助
 */

#ifndef __DRIVER_PS_H
#define __DRIVER_PS_H

#include "stm32f4xx.h"

#define SOP (uint8_t) 87 // start of package
#define EOP (uint8_t) 88 // end of package

typedef struct {
    uint8_t  data[34];
    uint16_t result[10];
    uint8_t  id;  // command id
    uint8_t  len; // command id
    uint32_t pid; // package id
} PsData_Type;

void Ps_Update(PsData_Type *PsData, uint8_t newByte);
void Ps_Parse_Header(PsData_Type *PsData, uint8_t value);
void Ps_Parse_Data(PsData_Type *PsData, uint8_t *data);
void Ps_Valid(PsData_Type *PsData, uint8_t crc);
void Ps_Reset(PsData_Type *PsData);

uint8_t CRC_Calculate(uint8_t *data, uint8_t len);
uint8_t CRC_Valid(uint8_t *data, uint8_t len, uint8_t crc);

#endif
