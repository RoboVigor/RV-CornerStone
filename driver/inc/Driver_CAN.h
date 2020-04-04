/**
 * @file Driver_CAN.h
 * @brief CAN通讯驱动
 * @note CAN接收在stm32f4xx_it.c中定义
 * @version 0.5
 * - Can_Send() 新增了电调标识符id参数,所有CAN发送只需要这一个函数就能够完成
 */

#ifndef __DRIVER_CAN_H
#define __DRIVER_CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "Driver_Protocol.h"

/**
 * @brief CAN发送8bit数据
 *
 * @param CANx  指定一个CAN口,CAN1或CAN2
 * @param id ID，值越低优先级越高(电调ID,0x200/0x1ff)
 * @param i_201 第1、2个bit(0x201地址电机给定电流值,范围-32768~32768)
 * @param i_202 第3、4个bit(0x202地址电机给定电流值,范围-32768~32768)
 * @param i_203 第5、6个bit(0x203地址电机给定电流值,范围-32768~32768)
 * @param i_204 第7、8个bit(0x204地址电机给定电流值,范围-32768~32768)
 */
void Can_Send(CAN_TypeDef *CANx, int16_t stdId, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);

/**
 * @brief CAN发送数据
 *
 * @param CANx  指定一个CAN口,CAN1或CAN2
 * @param Msg 需要传输的数据地址
 * @param dataLength 数据长度
 */
void Can_Send_Msg(CAN_TypeDef *CANx, Protocol_Data_Type *Msg, uint16_t dataLength);

/**
 * @brief CAN接收数据
 *
 * @param CanRxData CAN口接收到数据包
 * @param Msg 需要接收的数据地址
 */
void Can_Receive_Msg(CanRxMsg *CanRxData, Protocol_Data_Type *Msg);

#endif
