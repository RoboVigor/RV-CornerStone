#include "Driver_Protocol.h"
#include "Driver_Bridge.h"
#include "vegmath.h"
#include <string.h>

ProtocolInfo_Type ProtocolInfoList[] = PROTOCOL_INFO_LIST;
uint8_t           ProtocolInfoListSize;

ProtocolInfo_Type *Protocol_Get_Info_Handle(uint16_t id) {
    int i = 0;
    for (i = 0; i < ProtocolInfoListSize; i++) {
        if (ProtocolInfoList[i].id == id) {
            return &ProtocolInfoList[i];
        }
    }
    return &ProtocolInfoList[ProtocolInfoListSize - 1];
}

void Protocol_Init(Node_Type *node, ProtocolData_Type *protocolData) {
    uint16_t i = 0, sum = 0;

    // 初始化节点
    node->state        = STATE_IDLE;
    node->step         = STEP_HEADER_SOF;
    node->data         = protocolData->data;
    node->protocolData = protocolData;

    // 初始化协议
    if (ProtocolInfoList[1].offset == 0) {
        ProtocolInfoListSize = sizeof(ProtocolInfoList) / sizeof(ProtocolInfo_Type);
        for (i = 0; i < ProtocolInfoListSize; i++) {
            ProtocolInfoList[i].offset = sum;
            sum += ProtocolInfoList[i].length;
        }
    }
}

void Protocol_Update(Node_Type *node) {
    int i = 0;

    node->state = STATE_WORK;

    for (i = 0; i < Protocol_Buffer_Length; i++) {
        Protocol_Unpack(node, node->receiveBuf[i]);
    }
}

uint16_t Protocol_Pack(Node_Type *node, uint16_t id) {
    ProtocolInfo_Type *protocolInfo;
    uint16_t           dataLength, offset, dataCRC16, i = 0, index = 0, packetId = id, dataId = id;
    uint8_t *          begin_p;
    uint8_t *          send_p;

    node->state = STATE_WORK;

    // Protocol Info
    protocolInfo = Protocol_Get_Info_Handle(id);
    offset       = protocolInfo->offset;
    dataLength   = protocolInfo->length;

    // TaoWaMode
    if (id >= 0xF000) {
        packetId = 0x301;
        dataId   = id & 0x0FFF;
    }

    // Header SOF
    node->sendBuf[index++] = PROTOCOL_HEADER;

    // Data Length
    node->sendBuf[index++] = (dataLength) &0xff;
    node->sendBuf[index++] = (dataLength) >> 8;

    // Frame SEQ
    node->sendBuf[index++]++;

    // Header CRC8
    node->sendBuf[index++] = Get_CRC8_Check_Sum(node->sendBuf, PROTOCOL_HEADER_SIZE - 1);

    // Cmd ID
    node->sendBuf[index++] = (packetId) &0xff;
    node->sendBuf[index++] = (packetId) >> 8;

    // Clear
    for (i = PROTOCOL_HEADER_CMDID_LEN; i < Protocol_Buffer_Length; i++) {
        node->sendBuf[i] = 0x00;
    }

    // Data
    begin_p = node->data + offset;
    for (i = 0; i < dataLength; i++) {
        node->sendBuf[index++] = *(begin_p + i);
    }

    // Data CRC16
    dataCRC16              = Get_CRC16_Check_Sum(node->sendBuf, PROTOCOL_HEADER_CMDID_LEN + dataLength);
    node->sendBuf[index++] = (dataCRC16) &0xff;
    node->sendBuf[index++] = (dataCRC16) >> 8;

    send_p = node->sendBuf;
    for (i = 0; i < index; i++) {
        *send_p++ = node->sendBuf[i];
    }

    return dataLength;
}

void Protocol_Unpack(Node_Type *node, uint8_t byte) {

    node->state = STATE_WORK;

    switch (node->step) {
    case STEP_HEADER_SOF: {
        if (byte == PROTOCOL_HEADER) {
            node->packet[node->index++] = byte;
            node->step                  = STEP_LENGTH_LOW;
        } else {
            node->index = 0;
        }
    } break;

    case STEP_LENGTH_LOW: {
        node->dataLength            = byte;
        node->packet[node->index++] = byte;
        node->step                  = STEP_LENGTH_HIGH;
    } break;

    case STEP_LENGTH_HIGH: {
        node->dataLength |= byte << 8;
        node->packet[node->index++] = byte;
        if (node->dataLength < 114) {
            node->step = STEP_FRAME_SEQ;
        } else {
            node->step  = STEP_HEADER_SOF;
            node->index = 0;
        }
    } break;

    case STEP_FRAME_SEQ: {
        node->packet[node->index++] = byte;
        node->step                  = STEP_HEADER_CRC8;
    } break;

    case STEP_HEADER_CRC8: {
        node->packet[node->index++] = byte;
        if (Verify_CRC8_Check_Sum(node->packet, PROTOCOL_HEADER_SIZE)) {
            node->step = STEP_DATA_CRC16;
        } else {
            node->index = 0;
            node->step  = STEP_HEADER_SOF;
        }
    } break;

    case STEP_DATA_CRC16: {
        if (node->index < (PROTOCOL_HEADER_CRC_CMDID_LEN + node->dataLength)) {
            node->packet[node->index++] = byte;
        }
        if (node->index >= (PROTOCOL_HEADER_CRC_CMDID_LEN + node->dataLength)) {
            node->packet[node->index++] = byte;
            node->index                 = 0;
            node->step                  = STEP_HEADER_SOF;
            if (Verify_CRC16_Check_Sum(node->packet, PROTOCOL_HEADER_CRC_CMDID_LEN + node->dataLength)) {
                Protocol_Load(node);
            }
        }
    } break;

    default: {
        node->step  = STEP_HEADER_SOF;
        node->index = 0;
    } break;
    }
}

void Protocol_Load(Node_Type *node) {
    ProtocolInfo_Type *protocolInfo;
    uint16_t           i = 0, dataId = 0, dataLength, offset;
    uint8_t *          begin_p;

    // seq
    node->receiveSeq = node->packet[3];

    // id
    node->id = (node->packet[PROTOCOL_HEADER_SIZE + 1] << 8) + node->packet[PROTOCOL_HEADER_SIZE];
    if (node->id == 0x301) {
        dataId = (node->packet[PROTOCOL_HEADER_CMDID_LEN + 1] << 8) + node->packet[PROTOCOL_HEADER_CMDID_LEN];
    } else {
        dataId = node->id;
    }

    // get memory address
    protocolInfo = Protocol_Get_Info_Handle(dataId);
    offset       = protocolInfo->offset;
    dataLength   = protocolInfo->length;

    // load
    begin_p = node->data + offset;
    for (i = 0; i < node->dataLength; i++) {
        *(begin_p + i) = node->packet[PROTOCOL_HEADER_CMDID_LEN + i];
    }
}
