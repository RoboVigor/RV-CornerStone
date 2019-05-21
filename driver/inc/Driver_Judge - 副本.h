#ifndef __DRIVER_JUDGESYS_H
#define __DRIVER_JUDGESYS_H

#include "stm32f4xx.h"

#define JudgeBufferLength 32

#define JudgePackLength_0001 3
#define JudgePackLength_0002 1
#define JudgePackLength_0003 2
#define JudgePackLength_0101 4
#define JudgePackLength_0102 3
#define JudgePackLength_0103 2
#define JudgePackLength_0201 15
#define JudgePackLength_0202 14
#define JudgePackLength_0203 16
#define JudgePackLength_0204 1
#define JudgePackLength_0205 3
#define JudgePackLength_0206 1
#define JudgePackLength_0207 6
#define JudgePackLength_0301 10

#define JudgeFrameHeader 0xA5

typedef union {
    uint8_t U[4];
    float   F;
    int     I;
} FormatTrans;

typedef __packed struct {
    uint16_t stageRemainTime;
    uint8_t  gameProcess;
    uint8_t  robotLevel;
    uint16_t remainHP;
    uint16_t maxHP;
} extGameRobotState_t;

typedef __packed struct {
    uint8_t  bulletType;
    uint8_t  bulletFreq;
    float    bulletSpeed;
    uint16_t bullet_int;
} extShootData_t;

typedef __packed struct {
    float    chassisVolt;
    uint16_t Volt_int;
    float    chassisCurrent;
    uint16_t Current_int;
    float    chassisPower;
    uint16_t Power_int;
    float    chassisPowerBuffer;
    uint16_t PowerBuffer_int;
    uint16_t shooterHeat0;
    uint16_t shooterHeat1;
} extPowerHeatData_t;

typedef struct {
    uint8_t             buf[JudgeBufferLength];
    extGameRobotState_t robotState;
    extShootData_t      shootData;
    extPowerHeatData_t  powerHeatData;
} Judge_Type;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int  Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void          Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t      Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t      Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void          Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void          Judge_Init(Judge_Type *Judge);
void          Judge_Decode(Judge_Type *Judge);
#endif
