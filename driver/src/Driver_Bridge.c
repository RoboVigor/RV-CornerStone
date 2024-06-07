#include "Driver_Bridge.h"
#pragma pack(8)

void Bridge_Bind(Bridge_Type *bridge, uint8_t type, uint32_t deviceID, void *handle) {
    if (IS_MOTOR) {
        MOTOR = handle;
    } else {
        Node_Type *node = handle;
        if (IS_CAN) {
            CAN_NODE = node;
        } else {
            BSP_DMA_Init(USARTx_Rx, node->receiveBuf, Protocol_Buffer_Length);
            BSP_DMA_Init(USARTx_Tx, node->sendBuf, Protocol_Buffer_Length);
            USART_NODE = node;
        }
        node->deviceID   = deviceID;
        node->bridgeType = type;
    }
}

void Bridge_Receive_USART(Bridge_Type *bridge, uint8_t type, uint32_t deviceID) {
    uint16_t   i, tmp, len;
    Node_Type *node = USART_NODE;

    // clear IDLE flag
    tmp = USARTx->DR;
    tmp = USARTx->SR;

    // disabe DMA
    DMA_Disable(USARTx_Rx);

    // unpack
    len = Protocol_Buffer_Length - DMA_Get_Data_Counter(USARTx_Rx);
    for (i = 0; i < len; i++) {
        node->isFirstByte = 1; // @todo: 了解USART DMA的工作方式, 更新该变量
        Protocol_Unpack(node, node->receiveBuf[i]);
    }

    // enable DMA
    DMA_Enable(USARTx_Rx, Protocol_Buffer_Length);
}

void Bridge_Receive_CAN(Bridge_Type *bridge, uint8_t type) {
    uint16_t   i;
    Node_Type *node;
    uint32_t   deviceID;
    CanRxMsg   CanRxData;

    // 读取数据
    CAN_Receive(type == CAN1_BRIDGE ? CAN1 : CAN2, CAN_FIFO0, &CanRxData);
    deviceID = CanRxData.StdId;

    // 安排数据
    if (IS_MOTOR) {
        Motor_Type *motor = MOTOR;
        Motor_Update(motor, CanRxData.Data);
    } else {
        for (i = 0; i < CanRxData.DLC; i++) {
            node              = CAN_NODE;
            node->isFirstByte = i == 0 ? 1 : 0;
            Protocol_Unpack(node, CanRxData.Data[i]);
        }
    }
}

void Bridge_Send_Motor(Bridge_Type *bridge, uint8_t safetyMode) {
    static CAN_TypeDef *Canx[2]          = {CAN1, CAN2};
    static uint16_t     Can_Send_Id[3]   = {0x200, 0x1ff, 0x2ff};
    static uint16_t     Can_ESC_Id[3][4] = {{0x201, 0x202, 0x203, 0x204}, {0x205, 0x206, 0x207, 0x208}, {0x209, 0x020a, 0x20b, 0x20c}};
    static int          isNotEmpty       = 0; // 同一发送ID下是否有电机
    static Motor_Type * motor;                // 根据i,j,k锁定电机
    static int16_t      currents[4];          // CAN发送电流
    int                 i, j, k;              // CAN序号 发送ID序号 电调ID序号
    int                 motorEnabled;
    uint32_t            deviceID;
    uint8_t             type;

    Bridge_Check_Motor_Watchdog(bridge); // 检查电机是否在线

    for (i = 0; i < 2; i++) {
        type = i == 0 ? CAN1_BRIDGE : CAN2_BRIDGE;
        for (j = 0; j < 3; j++) {
            isNotEmpty = 0;
            for (k = 0; k < 4; k++) {
                deviceID     = Can_ESC_Id[j][k];
                motor        = MOTOR;
                motorEnabled = motor && motor->inputEnabled && motor->online;
                currents[k]  = motorEnabled ? motor->input : 0;
                isNotEmpty   = isNotEmpty || motorEnabled;
            }
            if (isNotEmpty && !safetyMode) {
                Can_Send(Canx[i], Can_Send_Id[j], currents[0], currents[1], currents[2], currents[3]);
            } else if (isNotEmpty && safetyMode) {
                Can_Send(Canx[i], Can_Send_Id[j], 0, 0, 0, 0);
            }
        }
    }
}

void Bridge_Check_Motor_Watchdog(Bridge_Type *bridge) {
    int         i;
    Motor_Type *motor;
    int8_t      vegtableMotorId = -1;
    for (i = 0; i < 24; i++) {
        motor = bridge->motors[i];
        if (motor != 0 && xTaskGetTickCount() - motor->updatedAt > CAN_TIMEOUT) {
            vegtableMotorId = i;
            motor->online   = 0;
            break;
        }
    }
    if (vegtableMotorId != -1) {
        if (vegtableMotorId < 12) {
            LED_Set_Warning(0b11110000, vegtableMotorId + 1);
        } else {
            LED_Set_Warning(0b00001111, vegtableMotorId - 11);
        }
    } else {
        LED_Cancel_Warning();
    }
}

void Bridge_Release_Lock_USART(Bridge_Type *bridge, uint8_t type, uint32_t deviceID) {
    uint16_t   i, tmp, len;
    Node_Type *node = USART_NODE;
    node->sendLock  = 0;
}

uint8_t Bridge_Send_Protocol_Once(Node_Type *node, uint32_t commandID) {
    uint32_t deviceID = node->deviceID;
    uint16_t dataLength;
    uint8_t  type = node->bridgeType;
    // DMA_Type *dma  = ((uint64_t) DMA_Table) + USARTx_Tx * 0x24;

    if (node->sendLock) return 0;
    node->sendLock = 1;

    if (IS_CAN) {
        dataLength = Protocol_Pack(node, commandID);
        Can_Send_Msg(type == CAN1_BRIDGE ? CAN1 : CAN2, CAN_DEVICE_ID, node->sendBuf, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
    } else {
        // while (DMA_GetFlagStatus(dma->DMAx_Streamy, dma->DMA_FLAG_TCIFx) != SET) {
        //}
        DMA_Disable(USARTx_Tx);
        dataLength = Protocol_Pack(node, commandID);
        DMA_Enable(USARTx_Tx, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
        // while (DMA_GetFlagStatus(dma->DMAx_Streamy, dma->DMA_FLAG_TCIFx) != SET) {
        //}
    }

    node->sendLock = 0;
    return 1;
}

void Bridge_Send_Protocol(Node_Type *node, uint32_t commandID, uint16_t frequency) {
    ProtocolInfo_Type *protocolInfo = Protocol_Get_Info_Handle(commandID);
    char               taskName[24];
    sprintf(taskName, "Task_Send_Protocol_%3x", commandID);
    if (protocolInfo->taskHandle != 0) {
        vTaskDelete(protocolInfo->taskHandle);
    }
    protocolInfo->node      = node;
    protocolInfo->frequency = frequency;
    xTaskCreate(Task_Send_Protocol, taskName, 500, (void *) protocolInfo, 6, protocolInfo->taskHandle);
}

void Task_Send_Protocol(void *Parameters) {
    TickType_t         LastWakeTime = xTaskGetTickCount();
    ProtocolInfo_Type *protocolInfo = Parameters;
    uint8_t            isSuccess;
    while (1) {
        isSuccess = 0;
        while (!isSuccess) {
            isSuccess = Bridge_Send_Protocol_Once(protocolInfo->node, protocolInfo->id);
            vTaskDelayUntil(&LastWakeTime, 3);
        }
        // isSuccess = Bridge_Send_Protocol_Once(protocolInfo->node, protocolInfo->id);
        vTaskDelayUntil(&LastWakeTime, 1000.0 / protocolInfo->frequency);
    }
    vTaskDelete(NULL);
}