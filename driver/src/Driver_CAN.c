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
