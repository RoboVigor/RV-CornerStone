#include "Driver_Protocol.h"
#include "Driver_Bridge.h"
#include "vegmath.h"

uint8_t  ProtocolDataLengthArray[64] = {PROTOCOL_HOST_LENGTH_FUNCTION(COMMA), PROTOCOL_JUDGE_LENGTH_FUNCTION(COMMA), PROTOCOL_USER_LENGTH_FUNCTION(COMMA)};
uint16_t ProtocolDataIdArray[64]     = {PROTOCOL_HOST_ID_ARRAY, PROTOCOL_JUDGE_ID_ARRAY, PROTOCOL_USER_ID_ARRAY, 0};

extern Protocol_Data_Type ProtocolData;

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

void Protocol_Init(Protocol_Channel_Type *channel, Protocol_Data_Type *data) {
    channel->state = STATE_IDLE;
    channel->step  = STEP_HEADER_SOF;
    channel->data  = data->data;
}

void Protocol_Update(Protocol_Channel_Type *channel) {
    int i = 0;

    channel->state = STATE_WORK;

    for (i = 0; i < Protocol_Buffer_Length; i++) {
        Protocol_Unpack(channel, channel->receiveBuf[i]);
    }
}

uint16_t Protocol_Pack(Protocol_Channel_Type *channel, uint16_t id) {
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

    channel->state = STATE_WORK;

    // get memory address
    Protocol_Get_Packet_Info(id, &offset, &dataLength);

    // TaoWaMode
    if (id >= 0xF000) {
        packetId = 0x301;
        dataId   = id & 0x0FFF;
    }

    // Header SOF
    channel->sendBuf[index++] = PROTOCOL_HEADER;

    // Data Length
    channel->sendBuf[index++] = (dataLength) &0xff;
    channel->sendBuf[index++] = (dataLength) >> 8;

    // Frame SEQ
    channel->sendBuf[index++]++;

    // Header CRC8
    channel->sendBuf[index++] = Get_CRC8_Check_Sum(channel->sendBuf, PROTOCOL_HEADER_SIZE - 1, CRC8_INIT);

    // Cmd ID
    channel->sendBuf[index++] = (packetId) &0xff;
    channel->sendBuf[index++] = (packetId) >> 8;

    // Clear
    for (i = PROTOCOL_HEADER_CMDID_LEN; i < Protocol_Buffer_Length; i++) {
        channel->sendBuf[i] = 0x00;
    }

    // Data
    begin_p = channel->data + offset;
    for (i = 0; i < dataLength; i++) {
        channel->sendBuf[index++] = *(begin_p + i);
    }

    // Data CRC16
    dataCRC16                 = Get_CRC16_Check_Sum(channel->sendBuf, PROTOCOL_HEADER_CMDID_LEN + dataLength, CRC16_INIT);
    channel->sendBuf[index++] = (dataCRC16) &0xff;
    channel->sendBuf[index++] = (dataCRC16) >> 8;

    send_p = channel->sendBuf;
    for (i = 0; i < index; i++) {
        *send_p++ = channel->sendBuf[i];
    }

    return dataLength;
}

void Protocol_Unpack(Protocol_Channel_Type *channel, uint8_t byte) {

    channel->state = STATE_WORK;

    switch (channel->step) {
    case STEP_HEADER_SOF: {
        if (byte == PROTOCOL_HEADER) {
            channel->packet[channel->index++] = byte;
            channel->step                     = STEP_LENGTH_LOW;
        } else {
            channel->index = 0;
        }
    } break;

    case STEP_LENGTH_LOW: {
        channel->dataLength               = byte;
        channel->packet[channel->index++] = byte;
        channel->step                     = STEP_LENGTH_HIGH;
    } break;

    case STEP_LENGTH_HIGH: {
        channel->dataLength |= byte << 8;
        channel->packet[channel->index++] = byte;
        if (channel->dataLength < 114) {
            channel->step = STEP_FRAME_SEQ;
        } else {
            channel->step  = STEP_HEADER_SOF;
            channel->index = 0;
        }
    } break;

    case STEP_FRAME_SEQ: {
        channel->packet[channel->index++] = byte;
        channel->step                     = STEP_HEADER_CRC8;
    } break;

    case STEP_HEADER_CRC8: {
        channel->packet[channel->index++] = byte;
        if (Verify_CRC8_Check_Sum(channel->packet, PROTOCOL_HEADER_SIZE)) {
            channel->step = STEP_DATA_CRC16;
        } else {
            channel->index = 0;
            channel->step  = STEP_HEADER_SOF;
        }
    } break;

    case STEP_DATA_CRC16: {
        if (channel->index < (PROTOCOL_HEADER_CRC_CMDID_LEN + channel->dataLength)) {
            channel->packet[channel->index++] = byte;
        }
        if (channel->index >= (PROTOCOL_HEADER_CRC_CMDID_LEN + channel->dataLength)) {
            channel->packet[channel->index++] = byte;
            channel->index                    = 0;
            channel->step                     = STEP_HEADER_SOF;
            if (Verify_CRC16_Check_Sum(channel->packet, PROTOCOL_HEADER_CRC_CMDID_LEN + channel->dataLength)) {
                Protocol_Load(channel);
            }
        }
    } break;

    default: {
        channel->step  = STEP_HEADER_SOF;
        channel->index = 0;
    } break;
    }
}

void Protocol_Load(Protocol_Channel_Type *channel) {
    int      i;
    uint8_t *begin_p;
    uint16_t dataId = 0;
    uint16_t dataLength;
    uint16_t offset;

    // seq
    channel->seq = channel->packet[3];

    // id
    channel->id = (channel->packet[PROTOCOL_HEADER_SIZE + 1] << 8) + channel->packet[PROTOCOL_HEADER_SIZE];
    if (channel->id == 0x301) {
        dataId = (channel->packet[PROTOCOL_HEADER_CMDID_LEN + 1] << 8) + channel->packet[PROTOCOL_HEADER_CMDID_LEN];
    } else {
        dataId = channel->id;
    }

    // get memory address
    Protocol_Get_Packet_Info(dataId, &offset, &dataLength);

    // load
    begin_p = channel->data + offset;
    for (i = 0; i < channel->dataLength; i++) {
        *(begin_p + i) = channel->packet[PROTOCOL_HEADER_CMDID_LEN + i];
    }
}
