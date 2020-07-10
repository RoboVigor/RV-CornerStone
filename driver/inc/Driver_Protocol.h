#ifndef __DRIVER_PROTOCOL_H
#define __DRIVER_PROTOCOL_H

#pragma pack(1)

#include "stm32f4xx.h"

#define PLUS +
#define COMMA ,

#define Protocol_Buffer_Length 128

#define PROTOCOL_HEADER 0xA5
#define PROTOCOL_HEADER_SIZE 5
#define PROTOCOL_CMD_SIZE 2
#define PROTOCOL_CRC16_SIZE 2
#define PROTOCOL_PACK_0301_HEADER 6
#define PROTOCOL_PACK_0301_DATA_MAX 112
#define PROTOCOL_HEADER_CRC_LEN (PROTOCOL_HEADER_SIZE + PROTOCOL_CRC16_SIZE)
#define PROTOCOL_HEADER_CRC_CMDID_LEN (PROTOCOL_HEADER_SIZE + PROTOCOL_CRC16_SIZE + PROTOCOL_CMD_SIZE)
#define PROTOCOL_HEADER_CMDID_LEN (PROTOCOL_HEADER_SIZE + PROTOCOL_CMD_SIZE)

// clang-format off

#include "protocol_host.h"
#include "protocol_judge.h"
#include "protocol_user.h"

#define PROTOCOL_DATA_LENGTH         PROTOCOL_HOST_LENGTH_FUNCTION(PLUS)+PROTOCOL_JUDGE_LENGTH_FUNCTION(PLUS)+PROTOCOL_USER_LENGTH_FUNCTION(PLUS)
#define PROTOCOL_DATA_LENGTH_ARRAY  {PROTOCOL_DATA_LENGTH_FUNCTION(COMMA)}

// clang-format on

typedef union {
    uint8_t  U8[4];
    uint16_t U16[2];
    uint32_t U32;
    float    F;
    int16_t  I;
} format_trans_t;

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

typedef enum {
    STATE_IDLE = 0,
    STATE_WORK = 1,
} protocol_state_e;

typedef struct {
    uint8_t          sendBuf[Protocol_Buffer_Length];    // DMA发送缓存
    uint8_t          receiveBuf[Protocol_Buffer_Length]; // DMA接收缓存
    uint8_t          packet[Protocol_Buffer_Length];     // 有效字节数组
    unpack_step_e    step;                               // 当前解包步骤
    protocol_state_e state;                              // 当前工作状态
    uint16_t         index;                              // 当前包字节序
    uint16_t         dataLength;                         // 包数据长度
    uint16_t         seq;                                // 包序号
    uint16_t         id;                                 // 包编号
    uint8_t *        data;                               // 数据存放地址
} Protocol_Channel_Type;

typedef struct {
    union {
        struct {
            PROTOCOL_HOST_t  host;
            PROTOCOL_JUDGE_t judge;
            PROTOCOL_USER_t  user;
        };
        struct {
            uint8_t data[PROTOCOL_DATA_LENGTH];
        };
    };
} Protocol_Data_Type;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int  Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void          Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t      Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t      Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void          Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

void     Protocol_Get_Packet_Info(uint16_t id, uint16_t *offset, uint16_t *length);
void     Protocol_Init(Protocol_Channel_Type *channel, Protocol_Data_Type *data);
void     Protocol_Update(Protocol_Channel_Type *channel);
void     Protocol_Unpack(Protocol_Channel_Type *channel, uint8_t byte);
void     Protocol_Load(Protocol_Channel_Type *channel);
uint16_t Protocol_Pack(Protocol_Channel_Type *channel, uint16_t id);

#endif
