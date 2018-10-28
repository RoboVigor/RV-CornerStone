#ifndef __DRIVER_REFEREE_H
#define __DRIVER_REFEREE_H

#include "stm32f4xx.h"

#define JudgeBufferLength 32

#define JudgePackLength_1 17
#define JudgePackLength_3 15
#define JudgePackLength_4 29

#define JudgeFrameHeader 0xA5 //帧头

#ifdef __DRIVER_JUDGE_GLOBALS
#define __DRIVER_JUDGE_EXT
#else
#define __DRIVER_JUDGE_EXT extern
#endif

typedef __packed struct {
    uint16_t stageRemainTime;
    uint8_t  gameProcess;
    uint8_t  robotLevel;
    uint16_t remainHP;
    uint16_t maxHP;
} extGameRobotState_t; //机器人状态

typedef __packed struct {
    uint8_t  bulletType;
    uint8_t  bulletFreq;
    float    bulletSpeed;
    uint16_t bullet_int;
} extShootData_t; //射击信息

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
} extPowerHeatData_t; //功率热量数据

__DRIVER_JUDGE_EXT uint8_t JudgeDataBuffer[JudgeBufferLength];
__DRIVER_JUDGE_EXT extGameRobotState_t Judge_RobotState;
__DRIVER_JUDGE_EXT extShootData_t Judge_ShootData;
__DRIVER_JUDGE_EXT extPowerHeatData_t Judge_PowerHeatData;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int  Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void          Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t      Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t      Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void          Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void          Decode_JudgeData(void);
void          JudgeData_Init(void);

#endif
