/**
 * @brief CAN通讯驱动程序
 *
 * @note CAN接收在stm32f4xx_it.c中定义
 * @file Driver_CAN.c
 */

#include "Driver_CAN.h"

/**
 * @brief CAN发送数据给电调
 *
 * @param CANx  指定一个CAN口,CAN1或CAN2
 * @param id 电调ID,0x200(C620,820R)
 * @param i_201 0x201地址电机给定电流值,范围-32768~32768
 * @param i_202 0x202地址电机给定电流值,范围-32768~32768
 * @param i_203 0x203地址电机给定电流值,范围-32768~32768
 * @param i_204 0x204地址电机给定电流值,范围-32768~32768
 */
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
