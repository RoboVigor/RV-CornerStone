/**
 * @file Driver_Bridge.h
 * @brief 信息高速公路
 */
#ifndef __DRIVER_BRIDGE_H
#define __DRIVER_BRIDGE_H

#include "stm32f4xx_conf.h"
#include "stm32f4xx_can.h"
#include "Driver_Motor.h"
#include "Driver_Protocol.h"
#include "Driver_CAN.h"
#include "Driver_Bridge.h"
#include "Driver_BSP.h"
#include "config.h"

typedef enum { CAN1_BRIDGE = 0, CAN2_BRIDGE, USART_BRIDGE } bridge_type_e;

/**
 * @brief pointer array
 * @param  motor
 */
typedef struct {
    Motor_Type *motors[24];    // [0-11] CAN1, [12-23] CAN2
    Node_Type * canNodes[10];  // [0-4] CAN1 [0x500-0x504], [5-9] CAN2 [0x500-0x504]
    Node_Type * usartNodes[4]; // [0-4] USART [3,6,7,8]
} Bridge_Type;

typedef struct {
    uint8_t        bridge_index;
    uint8_t        Tx;
    uint8_t        Rx;
    USART_TypeDef *USARTx;
} usart_t;

static usart_t USART[] = {{},
                          {},
                          {},
                          {0, USART3_Tx, USART3_Rx, USART3_BASE},
                          {},
                          {},
                          {1, USART6_Tx, USART6_Rx, USART6_BASE},
                          {2, UART7_Tx, UART7_Rx, UART7_BASE},
                          {3, UART8_Tx, UART8_Rx, UART8_BASE}};

#define USARTx (USART[deviceID].USARTx)
#define USARTx_Tx (USART[deviceID].Tx)
#define USARTx_Rx (USART[deviceID].Rx)

#define IS_MOTOR (deviceID <= 0x2FF && deviceID >= 0x1FF)
#define IS_CAN (type != USART_BRIDGE)

#define MOTOR (bridge->motors[deviceID - 0x201 + (type == CAN2_BRIDGE ? 12 : 0)])
#define CAN_NODE (bridge->canNodes[deviceID - 0x500 + (type == CAN2_BRIDGE ? 5 : 0)])
#define USART_NODE (bridge->usartNodes[USART[deviceID].bridge_index])

/**
 * @brief 建立数据桥梁
 */
void Bridge_Bind(Bridge_Type *bridge, uint8_t type, uint32_t deviceID, void *handle);

/**
 * @brief 接收串口数据
 */
void Bridge_Receive_USART(Bridge_Type *bridge, uint8_t type, uint32_t deviceID);

/**
 * @brief 接收CAN数据
 * @todo  未测试
 */
void Bridge_Receive_CAN(Bridge_Type *bridge, uint8_t type);

/**
 * @brief 发送电机数据
 */
void Bridge_Send_Motor(Bridge_Type *bridge, uint8_t safetyMode);

/**
 * @brief 检查电机是否在线
 * @warning 如果超过CAN_TIMEOUT未收到数据则通过LED灯闪烁离线的电机ID
 *          靠近RM标志上部的一半灯条闪烁为CAN1, 否则为CAN2, 闪烁次数表示ID
 */
void Bridge_Check_Motor_Watchdog(Bridge_Type *bridge);

/**
 * @brief  释放发送锁
 */
void Bridge_Release_Lock_USART(Bridge_Type *bridge, uint8_t type, uint32_t deviceID);

/**
 * @brief  发送协议数据
 * @todo   未测试
 * @return 0=节点被占用, 1=成功
 */
uint8_t Bridge_Send_Protocol_Once(Node_Type *node, uint32_t commandID);

/**
 * @brief 按一定频率自动发送协议数据
 */
void Bridge_Send_Protocol(Node_Type *node, uint32_t commandID, uint16_t frequency);

/**
 * @brief 自动发送协议数据用的任务
 */
void Task_Send_Protocol(void *Parameters);
#endif