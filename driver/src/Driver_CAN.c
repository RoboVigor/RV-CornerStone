/**
 * @brief CAN通讯驱动程序
 *
 * @file Driver_CAN.c
 */

#define __DRIVER_CAN_GLOBALS
#include "Driver_CAN.h"

/**
 * @brief 发送数据给四个电机
 *
 * @param CANx  指定一个CAN口,CAN1或CAN2
 * @param i_201 0x201地址电机给定电流值,范围-32768~32768
 * @param i_202 0x202地址电机给定电流值,范围-32768~32768
 * @param i_203 0x203地址电机给定电流值,范围-32768~32768
 * @param i_204 0x204地址电机给定电流值,范围-32768~32768
 */
void Can_Set_CM_Current(CAN_TypeDef *CANx, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204) {
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE   = CAN_Id_Standard;
    tx_message.RTR   = CAN_RTR_Data;
    tx_message.DLC   = 0x08;

    tx_message.Data[0] = (uint8_t)(i_201 >> 8);
    tx_message.Data[1] = (uint8_t) i_201;
    tx_message.Data[2] = (uint8_t)(i_202 >> 8);
    tx_message.Data[3] = (uint8_t) i_202;
    tx_message.Data[4] = (uint8_t)(i_203 >> 8);
    tx_message.Data[5] = (uint8_t) i_203;
    tx_message.Data[6] = (uint8_t)(i_204 >> 8);
    tx_message.Data[7] = (uint8_t) i_204;

    do {
        if (CAN1->ESR) {
            printf("error: %x\r\n", CAN1->ESR);
            return;
            CAN1->MCR |= 0x02;
            CAN1->MCR &= 0xFD;
        }
    } while (!(CAN1->TSR & 0x1C000000));

    CAN_Transmit(CANx, &tx_message);
}

void CAN_Set_HookArmour_Speed(CAN_TypeDef *CANx, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204) {

    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE   = CAN_Id_Standard;
    tx_message.RTR   = CAN_RTR_Data;
    tx_message.DLC   = 0x08;

    tx_message.Data[0] = (uint8_t)(i_201 >> 8);
    tx_message.Data[1] = (uint8_t) i_201;
    tx_message.Data[2] = (uint8_t)(i_202 >> 8);
    tx_message.Data[3] = (uint8_t) i_202;
    tx_message.Data[4] = (uint8_t)(i_203 >> 8);
    tx_message.Data[5] = (uint8_t) i_203;
    tx_message.Data[6] = (uint8_t)(i_204 >> 8);
    tx_message.Data[7] = (uint8_t) i_204;

    // CAN故障重启
    do {
        if (CAN2->ESR) {
            CAN2->MCR |= 0x02;
            CAN2->MCR &= 0xFD;
        }
    } while (!(CAN2->TSR & 0x1C000000));

    CAN_Transmit(CANx, &tx_message);
}

/**
 * @brief  获取编码器的值
 * @param  void
 * @return void
 */
void CAN_Get_Encoder_Bias(volatile CANEncoder_Type *v) {
    //		int i;
    // v->ecdBias = 4333;  //保存初始编码器值作为偏差
    v->rawValue     = v->ecdBias;
    v->lastRawValue = v->ecdBias;
    v->roundCnt     = 0;
}

/**
 * @brief  电机机械角度解结解算,转子角度转成实际角度
 * @param  void
 * @return void
 */

void CAN_Update_Encoder_Data(volatile CANEncoder_Type *v, uint16_t agree) {
    int     i        = 0;
    int32_t temp_sum = 0;
    v->lastRawValue  = v->rawValue;
    v->rawValue      = agree;
    v->diff          = v->rawValue - v->lastRawValue;
    if (v->diff < -4200) //两次编码器的反馈值差别太大,表示圈数发生了改变
    {
        v->roundCnt++;
        v->ecdRawRate = v->diff + 8192;
    } else if (v->diff > 4200) {
        v->roundCnt--;
        v->ecdRawRate = v->diff - 8192;
    } else {
        v->ecdRawRate = v->diff;
    }
    //计算得到连续的编码器输出值
    v->ecdValue = v->rawValue + v->roundCnt * 8192;
    //计算得到角度值,范围正负无穷大
    v->ecdAngle               = (float) (v->rawValue - v->ecdBias) * 360 / 8192 + v->roundCnt * 360;
    v->rateBuf[v->bufCount++] = v->ecdRawRate;
    if (v->bufCount == RATE_BUF_SIZE) {
        v->bufCount = 0;
    }
    //计算速度平均值
    for (i = 0; i < RATE_BUF_SIZE; i++) {
        temp_sum += v->rateBuf[i];
    }
    v->filterRate = (int32_t)(temp_sum / RATE_BUF_SIZE);
}
