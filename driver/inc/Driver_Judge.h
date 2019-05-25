#ifndef __DRIVER_JUDGESYS_H
#define __DRIVER_JUDGESYS_H

#include "stm32f4xx.h"

#define JudgeBufferLength 128

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

#define REF_PROTOCOL_HEADER 0xA5
#define REF_PROTOCOL_HEADER_SIZE 5
#define REF_PROTOCOL_CMD_SIZE 2
#define REF_PROTOCOL_CRC16_SIZE 2
#define REF_HEADER_CRC_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

typedef union {
    uint8_t U[4];
    float   F;
    int     I;
} FormatTrans;

typedef struct {
    union {
        struct {
            uint8_t  robot_id;
            uint8_t  robot_level;
            uint16_t remain_HP;
            uint16_t max_HP;
            uint16_t shooter_heat0_cooling_rate;
            uint16_t shooter_heat0_cooling_limit;
            uint16_t shooter_heat1_cooling_rate;
            uint16_t shooter_heat1_cooling_limit;
            uint8_t  mains_power_gimbal_output : 1;
            uint8_t  mains_power_chassis_output : 1;
            uint8_t  mains_power_shooter_output : 1;
        };
        struct {
            uint8_t data[JudgePackLength_0201];
        };
    };
} ext_game_robot_state_t;

typedef struct {
    union {
        struct {
            uint16_t chassis_volt;
            uint16_t chassis_current;
            float    chassis_power;
            uint16_t chassis_power_buffer;
            uint16_t shooter_heat0;
            uint16_t shooter_heat1;
        };
        struct {
            uint8_t data[JudgePackLength_0202];
        };
    };
} ext_power_heat_data_t;

typedef struct {
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} frame_header_t;

typedef enum {
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef struct {
    uint8_t                buf[JudgeBufferLength];
    uint8_t                packet[JudgeBufferLength];
    unpack_step_e          step;
    uint16_t               index;
    uint16_t               dataLength;
    uint16_t               seq;
    uint16_t               id;
    ext_game_robot_state_t robotState;
    ext_power_heat_data_t  heatData;
} Judge_Type;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int  Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void          Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t      Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t      Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void          Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

void Judge_Init(Judge_Type *Judge);
void Judge_Update(Judge_Type *Judge);
void Judge_Decode(Judge_Type *Judge, uint8_t byte);
void Judge_Load(Judge_Type *Judge);
#endif
