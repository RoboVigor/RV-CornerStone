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

void Can_Send_Msg(CAN_TypeDef *CANx, Protocol_Data_Type *Msg, uint16_t id, uint16_t dataLength) {
    CanTxMsg Txmessage;
    int      i;
    uint8_t  mbox;

    Txmessage.StdId = id;
    Txmessage.IDE   = CAN_Id_Standard;
    Txmessage.RTR   = CAN_RTR_Data;
    Txmessage.DLC   = dataLength;

    for (i = 0; i < dataLength - 1; i++) {
        Txmessage.Data[i] = Msg->data[i];
    }

    mbox = CAN_Transmit(CANx, &message);

    while (CAN_TransmitStatus(CANx, mbox) == CAN_TxStatus_Failed) {
    }
}

uint16_t Can_Receive_Msg(CAN_TypeDef *CANx, Protocol_Data_Type *Msg) {
    CanRxMsg RxMessage;
    int      i;

    if (CAN_MessagePending(CANx, CAN_FIFO0) == 0) return 0;
    CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
    for (i = 0; i < RxMessage.DLC; i++)
        Msg->data[i] = RxMessage.Data[i];

    return RxMessage.DLC;
}
