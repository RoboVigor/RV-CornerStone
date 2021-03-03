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
} Protocol_state_e;

typedef struct {
    uint32_t deviceID;                           // 串口ID
    uint8_t  bridgeType;                         // 总线类型
    uint8_t  sendBuf[Protocol_Buffer_Length];    // DMA发送缓存
    uint8_t  receiveBuf[Protocol_Buffer_Length]; // DMA接收缓存
    uint8_t  packet[Protocol_Buffer_Length];     // 有效字节数组
    uint8_t  step;                               // 当前解包步骤
    uint8_t  state;                              // 当前工作状态
    uint16_t index;                              // 当前包字节序
    uint16_t dataLength;                         // 包数据长度
    uint16_t seq;                                // 包序号
    uint16_t id;                                 // 包编号
    uint8_t *data;                               // 数据存放地址
} Node_Type;

void     Protocol_Get_Packet_Info(uint16_t id, uint16_t *offset, uint16_t *length);
void     Protocol_Init(Node_Type *node, Protocol_Type *data);
void     Protocol_Update(Node_Type *node);
void     Protocol_Unpack(Node_Type *node, uint8_t byte);
void     Protocol_Load(Node_Type *node);
uint16_t Protocol_Pack(Node_Type *node, uint16_t id);

#endif
