#ifndef _DRIVER_VOFA_H
#define _DRIVER_VOFA_H

#include "stm32f4xx_conf.h"
#include "stm32f4xx_can.h"
#include "Driver_Motor.h"
#include "Driver_Protocol.h"
#include "Driver_CAN.h"
#include "Driver_Bridge.h"
#include "Driver_BSP.h"
#include "config.h"

#pragma pack(push, 1)
typedef struct{
    float   data[3];
    uint8_t tail[4];
    uint8_t data_num;
}Vofa_Debug_JustFloat_Type;
#pragma pack(pop)

#define vofaTail_t 0x7F800000
/* @note 注意此协议数据为十六进制float，debug赋值时记得强转为float
*/
void Vofa_Send(uint16_t frequency, uint16_t heapDeth, Node_Type * node, uint32_t commandID);
void Task_Vofa_Send(void * Parameters);

void Vofa_Debug_Init(Vofa_Debug_JustFloat_Type *vofa,uint32_t deviceID,uint8_t vofa_data_num);
void Vofa_Debug_Send(uint32_t deviceID);
#endif 
