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

#include "protocol.h"
#include "config.h"

// clang-format on

typedef union {
    uint8_t  U8[4];
    uint16_t U16[2];
    uint32_t U32;
    float    F;
    int16_t  I;
} Transformer_Type;

typedef enum {
    STEP_HEADER_SOF  = 0,
    STEP_LENGTH_LOW  = 1,
    STEP_LENGTH_HIGH = 2,
    STEP_FRAME_SEQ   = 3,
    STEP_HEADER_CRC8 = 4,
    STEP_DATA_CRC16  = 5,
} Unpack_step_e;

typedef enum {
    STATE_IDLE = 0,
    STATE_WORK = 1,
} protocol_state_e;

typedef struct {
    uint8_t          sendBuf[Protocol_Buffer_Length];    // DMA发送缓存
    uint8_t          receiveBuf[Protocol_Buffer_Length]; // DMA接收缓存
    uint8_t          packet[Protocol_Buffer_Length];     // 有效字节数组
    uint8_t    step;                               // 当前解包步骤
    protocol_state_e state;                              // 当前工作状态
    uint16_t         index;                              // 当前包字节序
    uint16_t         dataLength;                         // 包数据长度
    uint16_t         seq;                                // 包序号
    uint16_t         id;                                 // 包编号
    uint8_t *        data;                               // 数据存放地址
} Protocol_Channel_Type;

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int  Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void          Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t      Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t      Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void          Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

void     Protocol_Get_Packet_Info(uint16_t id, uint16_t *offset, uint16_t *length);
void     Protocol_Init(Protocol_Channel_Type *channel, Protocol_Type *data);
void     Protocol_Update(Protocol_Channel_Type *channel);
void     Protocol_Unpack(Protocol_Channel_Type *channel, uint8_t byte);
void     Protocol_Load(Protocol_Channel_Type *channel);
uint16_t Protocol_Pack(Protocol_Channel_Type *channel, uint16_t id);

#endif
