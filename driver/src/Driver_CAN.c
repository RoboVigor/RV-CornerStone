#include "Driver_CAN.h"

void Can_Send(CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204) {
    CanTxMsg CanTxData;

    CanTxData.StdId = id;
    CanTxData.IDE   = CAN_Id_Standard;
    CanTxData.RTR   = CAN_RTR_Data;
    CanTxData.DLC   = 0x08;

    CanTxData.Data[0] = (uint8_t)(i_201 >> 8);
    CanTxData.Data[1] = (uint8_t) i_201;
    CanTxData.Data[2] = (uint8_t)(i_202 >> 8);
    CanTxData.Data[3] = (uint8_t) i_202;
    CanTxData.Data[4] = (uint8_t)(i_203 >> 8);
    CanTxData.Data[5] = (uint8_t) i_203;
    CanTxData.Data[6] = (uint8_t)(i_204 >> 8);
    CanTxData.Data[7] = (uint8_t) i_204;

    do {
        if (CANx->ESR) {
            // 可以在这里输出ESR来查看CAN错误
            CANx->MCR |= 0x02;
            CANx->MCR &= 0xFD;
        }
    } while (!(CANx->TSR & 0x1C000000));

    CAN_Transmit(CANx, &CanTxData);
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
