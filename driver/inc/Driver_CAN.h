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
void Can_Send(CAN_TypeDef *CANx, int16_t stdId, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);

#endif
