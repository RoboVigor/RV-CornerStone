#include "Driver_Protocol.h"
#include "Driver_Bridge.h"
#include "vegmath.h"
#include "tasks.h"
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
    node->step         = STEP_SOF;
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

int16_t Protocol_Pack(Node_Type *node, uint16_t id) {
    ProtocolInfo_Type *protocolInfo;
    uint16_t           dataLength, offset, dataCRC16, i = 0, index = 0, packetId = id, dataId = id;
    uint8_t *          begin_p;
    uint8_t *          send_p;

    node->state = STATE_WORK;

    // Protocol Info
    protocolInfo = Protocol_Get_Info_Handle(id);
    offset       = protocolInfo->offset;
    dataLength   = protocolInfo->length;

    // Protocol Not Found
    if (protocolInfo->id != id) {
        return -1;
    }

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
    if (node->bridgeType == USART_BRIDGE) {
        node->sendBuf[index++] = Get_CRC8_Check_Sum(node->sendBuf, PROTOCOL_HEADER_SIZE - 1);
    }

    // Cmd ID
    node->sendBuf[index++] = (packetId) &0xff;
    node->sendBuf[index++] = (packetId) >> 8;

    // Clear
    for (i = index; i < Protocol_Buffer_Length; i++) {
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

    node->sendSeq = node->sendBuf[3];

    return dataLength;
}

void Protocol_Unpack(Node_Type *node, uint8_t byte) {

    node->state = STATE_WORK;

    switch (node->step) {
    case STEP_SOF: {
        if (byte == PROTOCOL_HEADER && node->isFirstByte) {
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
            node->step = STEP_SEQ;
        } else {
            node->step  = STEP_SOF;
            node->index = 0;
        }
    } break;

    case STEP_SEQ: {
        node->receiveSeq            = byte;
        node->packet[node->index++] = byte;
        if (node->bridgeType == USART_BRIDGE || (node->deviceID == 0x500)) {
            node->step = STEP_CRC8;
        } else {
            node->step = STEP_ID_LOW;
        }
    } break;

    case STEP_CRC8: {
        node->packet[node->index++] = byte;
        if (Verify_CRC8_Check_Sum(node->packet, PROTOCOL_HEADER_SIZE)) {
            node->step = STEP_ID_LOW;
        } else {
            node->step  = STEP_SOF;
            node->index = 0;
        }
    } break;

    case STEP_ID_LOW: {
        node->id                    = byte;
        node->packet[node->index++] = byte;
        node->step                  = STEP_ID_HIGH;
    } break;

    case STEP_ID_HIGH: {
        node->id |= byte << 8;
        node->packet[node->index++]     = byte;
        node->protocolInfo              = Protocol_Get_Info_Handle(node->id);
        node->protocolInfo->receiveSeq  = node->receiveSeq;
        node->protocolInfo->receiveTime = xTaskGetTickCount();
        // Protocol Not Found
        if (node->protocolInfo->id != node->id) {
            node->step  = STEP_SOF;
            node->index = 0;
            break;
        }
        // receive or throw the packet
        if (node->protocolInfo->receive) {
            node->step = STEP_DATA;
        } else {
            node->waitCount = node->dataLength + PROTOCOL_CRC16_SIZE;
            node->step      = STEP_WAIT;
        }
    } break;

    case STEP_DATA: {
        node->packet[node->index++] = byte;
        if (node->index == PROTOCOL_HEADER_CRC_CMDID_LEN + node->dataLength) {
            node->step = STEP_CRC16;
        } else {
            break;
        }
    };

    case STEP_CRC16: {
        if (Verify_CRC16_Check_Sum(node->packet, PROTOCOL_HEADER_CRC_CMDID_LEN + node->dataLength)) {
            node->step = STEP_LOAD;
        } else {
            node->step  = STEP_SOF;
            node->index = 0;
            break;
        }
    };

    case STEP_LOAD: {
        uint8_t *begin_p = node->data + node->protocolInfo->offset;
        uint16_t i;
        for (i = 0; i < node->dataLength; i++) {
            *(begin_p + i) = node->packet[PROTOCOL_HEADER_CMDID_LEN + i];
        }
        node->protocolInfo->receiveCount += 1;
        node->step  = STEP_SOF;
        node->index = 0;
    } break;

    case STEP_WAIT: {
        node->waitCount -= 1;
        if (node->waitCount == 0) {
            node->step  = STEP_SOF;
            node->index = 0;
        }
    } break;

    default: {
        node->step  = STEP_SOF;
        node->index = 0;
    } break;
    }
}