#include "Driver_Protocol.h"
#include "Driver_Bridge.h"
#include "vegmath.h"

uint8_t  ProtocolDataLengthArray[64] = ProtocolDataLengthList;
uint16_t ProtocolDataIdArray[64]     = ProtocolDataIdList;

void Protocol_Get_Packet_Info(uint16_t id, uint16_t *offset, uint16_t *length) {
    int i   = 0;
    *offset = 0;
    *length = 0;
    while (ProtocolDataIdArray[i] != 0) {
        if (ProtocolDataIdArray[i] == id) {
            *length = ProtocolDataLengthArray[i];
            return;
        } else {
            *offset += ProtocolDataLengthArray[i];
            i++;
        }
    }
    return;
}

void Protocol_Init(Node_Type *node, Protocol_Type *data) {
    node->state = STATE_IDLE;
    node->step  = STEP_HEADER_SOF;
    node->data  = data->data;
}

void Protocol_Update(Node_Type *node) {
    int i = 0;

    node->state = STATE_WORK;

    for (i = 0; i < Protocol_Buffer_Length; i++) {
        Protocol_Unpack(node, node->receiveBuf[i]);
    }
}

uint16_t Protocol_Pack(Node_Type *node, uint16_t id) {
    int      i;
    uint16_t index    = 0;
    uint16_t packetId = id;
    uint16_t dataId   = id;
    uint8_t *begin_p;
    uint8_t *send_p;
    uint16_t dataLength;
    uint16_t offset;
    uint8_t  CRC8_INIT  = 0xff;
    uint16_t CRC16_INIT = 0xffff;
    uint16_t dataCRC16;

    node->state = STATE_WORK;

    // get memory address
    Protocol_Get_Packet_Info(id, &offset, &dataLength);

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
    node->sendBuf[index++] = Get_CRC8_Check_Sum(node->sendBuf, PROTOCOL_HEADER_SIZE - 1, CRC8_INIT);

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
    dataCRC16              = Get_CRC16_Check_Sum(node->sendBuf, PROTOCOL_HEADER_CMDID_LEN + dataLength, CRC16_INIT);
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
    int      i;
    uint8_t *begin_p;
    uint16_t dataId = 0;
    uint16_t dataLength;
    uint16_t offset;

    // seq
    node->seq = node->packet[3];

    // id
    node->id = (node->packet[PROTOCOL_HEADER_SIZE + 1] << 8) + node->packet[PROTOCOL_HEADER_SIZE];
    if (node->id == 0x301) {
        dataId = (node->packet[PROTOCOL_HEADER_CMDID_LEN + 1] << 8) + node->packet[PROTOCOL_HEADER_CMDID_LEN];
    } else {
        dataId = node->id;
    }

    // get memory address
    Protocol_Get_Packet_Info(dataId, &offset, &dataLength);

    // load
    begin_p = node->data + offset;
    for (i = 0; i < node->dataLength; i++) {
        *(begin_p + i) = node->packet[PROTOCOL_HEADER_CMDID_LEN + i];
    }
}
