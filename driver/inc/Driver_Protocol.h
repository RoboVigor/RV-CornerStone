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

#include "FreeRTOS.h"
#include "task.h"
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
    STEP_SOF,
    STEP_LENGTH_LOW,
    STEP_LENGTH_HIGH,
    STEP_SEQ,
    STEP_CRC8,
    STEP_ID_LOW,
    STEP_ID_HIGH,
    STEP_DATA,
    STEP_CRC16,
    STEP_LOAD,
    STEP_WAIT
} Unpack_step_e;

typedef enum {
    STATE_IDLE = 0,
    STATE_WORK = 1,
} Protocol_state_e;

typedef struct {
    // 协议定义
    uint16_t id;     // 编号
    uint16_t length; // 数据段长度
    // 接收相关
    uint16_t   receive;        // 是否接收并解包该协议
    uint16_t   receiveCount;   // 接收计数
    uint16_t   receiveSeq;     // 接收包序号
    uint16_t   lastReceiveSeq; // (仅供用户使用)上次接收序号
    TickType_t receiveTime;    // 接收时间
    // 发送相关
    void *       node;       // 发送节点
    float        frequency;  // 发送频率 (实际发送频率会略低一些)
    TaskHandle_t taskHandle; // 发送任务
    // 存储相关
    uint16_t offset; // 寄存器偏移
} ProtocolInfo_Type;

typedef struct {
    // Bridge相关
    uint8_t  bridgeType; // 总线类型
    uint32_t deviceID;   // USART编号(3,6,7,8) / 电机电调ID / CAN设备(0x500-0x504)
    // 存储相关
    ProtocolData_Type *protocolData;
    ProtocolInfo_Type *protocolInfo;
    // 发送相关
    uint8_t  sendBuf[Protocol_Buffer_Length]; // DMA发送缓存
    uint16_t sendSeq;                         // 已发送包序号
    uint8_t  sendLock;                        // 发送锁
    // 接收相关
    uint8_t  receiveBuf[Protocol_Buffer_Length]; // DMA接收缓存
    uint8_t  packet[Protocol_Buffer_Length];     // 有效字节数组
    uint8_t  step;                               // 当前解包步骤
    uint8_t  state;                              // 当前工作状态
    uint16_t index;                              // 当前包字节序
    uint16_t dataLength;                         // 包数据长度
    uint16_t receiveSeq;                         // 包序号
    uint16_t id;                                 // 包编号
    uint8_t *data;                               // 数据存放地址
    uint16_t waitCount;                          // 需要丢弃的字节数
    uint16_t isFirstByte;                        // 当前字节是否为包的第一位 @todo: 未测试
} Node_Type;

ProtocolInfo_Type *Protocol_Get_Info_Handle(uint16_t id);
void               Protocol_Init(Node_Type *node, ProtocolData_Type *data);
void               Protocol_Unpack(Node_Type *node, uint8_t byte);
int16_t            Protocol_Pack(Node_Type *node, uint16_t id);

#endif
