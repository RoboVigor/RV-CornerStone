#include "Driver_CAN.h"

void Can_Send(CAN_TypeDef *CANx, int16_t id, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204) {
    CanTxMsg CanTxData;
    uint8_t  mailBox;

    if (id <= 0x7ff) {
        CanTxData.StdId = id;
        CanTxData.IDE   = CAN_Id_Standard;
    } else {
        CanTxData.ExtId = id;
        CanTxData.IDE   = CAN_Id_Extended;
    }

    CanTxData.RTR = CAN_RTR_Data;
    CanTxData.DLC = 0x08;

    CanTxData.Data[0] = (uint8_t)(i_201 >> 8);
    CanTxData.Data[1] = (uint8_t) i_201;
    CanTxData.Data[2] = (uint8_t)(i_202 >> 8);
    CanTxData.Data[3] = (uint8_t) i_202;
    CanTxData.Data[4] = (uint8_t)(i_203 >> 8);
    CanTxData.Data[5] = (uint8_t) i_203;
    CanTxData.Data[6] = (uint8_t)(i_204 >> 8);
    CanTxData.Data[7] = (uint8_t) i_204;

    // do {
    //     if (CANx->ESR) {
    //         // 可以在这里输出ESR来查看CAN错误
    //         CANx->MCR |= 0x02;
    //         CANx->MCR &= 0xFD;
    //     }
    // } while (!(CANx->TSR & 0x1C000000));

    mailBox = CAN_Transmit(CANx, &CanTxData);

    while (CAN_TransmitStatus(CANx, mailBox) != CAN_TxStatus_Ok) {
    }
}

void Can_Send_Msg(CAN_TypeDef *CANx, int16_t id, uint8_t *sendBuf, uint16_t length) {
    int data[4];
    int i;

    for (i = 0; i < length / 8 + 1; i++) {
        data[0] = *(sendBuf + 8 * i) << 8 | *(sendBuf + 8 * i + 1);
        data[1] = *(sendBuf + 8 * i + 2) << 8 | *(sendBuf + 8 * i + 3);
        data[2] = *(sendBuf + 8 * i + 4) << 8 | *(sendBuf + 8 * i + 5);
        data[3] = *(sendBuf + 8 * i + 6) << 8 | *(sendBuf + 8 * i + 7);
        Can_Send(CANx, id, data[0], data[1], data[2], data[3]);
        // delay_ms(100); //调试时用，防止发送太快，串口读取不到
    }
}
