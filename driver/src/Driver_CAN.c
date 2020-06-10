#include "Driver_CAN.h"

void Can_Send(CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204) {
    CanTxMsg message;
    uint8_t  mailBox;
    uint16_t i = 0;

    message.StdId = id;
    message.IDE   = CAN_Id_Standard;
    message.RTR   = CAN_RTR_Data;
    message.DLC   = 0x08;

    message.Data[0] = (uint8_t)(i_201 >> 8);
    message.Data[1] = (uint8_t) i_201;
    message.Data[2] = (uint8_t)(i_202 >> 8);
    message.Data[3] = (uint8_t) i_202;
    message.Data[4] = (uint8_t)(i_203 >> 8);
    message.Data[5] = (uint8_t) i_203;
    message.Data[6] = (uint8_t)(i_204 >> 8);
    message.Data[7] = (uint8_t) i_204;

    mailBox = CAN_Transmit(CANx, &message);

    while (CAN_TransmitStatus(CANx, mailBox) == CAN_TxStatus_Failed && i != 0xff) {
        i++;
    }
}

void Can_Send_Msg(CAN_TypeDef *CANx, Protocol_Type *Protocol, uint16_t id, uint16_t length) {
    int      data[4];
    uint16_t dataLength;
    int      i;

    dataLength = length - PROTOCOL_HEADER_CRC_CMDID_LEN;
    Protocol_Pack(Protocol, dataLength, id);
    for (i = 0; i < length / 8 + 1; i++) {
        data[0] = Protocol->sendBuf[8 * i] << 8 | Protocol->sendBuf[8 * i + 1];
        data[1] = Protocol->sendBuf[8 * i + 2] << 8 | Protocol->sendBuf[8 * i + 3];
        data[2] = Protocol->sendBuf[8 * i + 4] << 8 | Protocol->sendBuf[8 * i + 5];
        data[3] = Protocol->sendBuf[8 * i + 6] << 8 | Protocol->sendBuf[8 * i + 7];
        Can_Send(CANx, id, data[0], data[1], data[2], data[3]);
    }
}
