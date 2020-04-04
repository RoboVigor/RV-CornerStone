#include "Driver_CAN.h"

void Can_Send(CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204) {
    CanTxMsg message;
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

    do {
        if (CANx->ESR) {
            // 可以在这里输出ESR来查看CAN错误
            CANx->MCR |= 0x02;
            CANx->MCR &= 0xFD;
        }
    } while (!(CANx->TSR & 0x1C000000));

    CAN_Transmit(CANx, &message);
}

void Can_Send_Msg(CAN_TypeDef *CANx, Protocol_Data_Type *Msg, uint16_t dataLength) {
    int data[4];
    int id = 0x300;
    int i;

    for (i = 0; i < dataLength / 4; i++) {
        data[0] = id;
        data[1] = dataLength;
        data[2] = Msg->data_i[2 * i];
        data[3] = Msg->data_i[2 * i + 1];
        Can_Send(CANx, id, data[0], data[1], data[2], data[3]);
        id++;
    }
}

void Can_Receive_Msg(CanRxMsg *CanRxData, Protocol_Data_Type *Msg) {
    int id;
    int dataLength;
    int i;

    id         = (short) ((int) CanRxData->Data[0] << 8 | CanRxData->Data[1]);
    dataLength = (short) ((int) CanRxData->Data[2] << 8 | CanRxData->Data[3]);
    for (i = 0; i < 4; i++) {
        Msg->data[4 * (id - 0x300) + i] = CanRxData->Data[4 + i];
    }
}
