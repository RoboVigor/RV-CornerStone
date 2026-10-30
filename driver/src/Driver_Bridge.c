#include "Driver_Bridge.h"
#include "handle.h"
#include "gimbal_tuning.h"
#pragma pack(8)

void Bridge_Bind(Bridge_Type *bridge, uint8_t type, uint32_t deviceID, void *handle) {
    if (IS_MOTOR) {
        MOTOR = handle;
    }else if(deviceID <= 0x188 && deviceID >= 0x181){
        deviceID = deviceID - 0x180 + 0x200 + 12;
        MOTOR = handle;
    } 
    else {
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
#if GIMBAL_TUNING_ENABLED
    // This tuning target uses standard data frames on its motor-only CAN1 bus.
    // Do not read StdId from an extended frame where the vendor leaves it unset.
    if(type==CAN1_BRIDGE && (CanRxData.IDE!=CAN_Id_Standard || CanRxData.RTR!=CAN_RTR_Data)) return;
#endif
    deviceID = CanRxData.StdId;
#if GIMBAL_TUNING_ENABLED
    if(type==CAN1_BRIDGE && (deviceID==0x205 || deviceID==0x206) &&
       (CanRxData.DLC!=8 || ((uint16_t)CanRxData.Data[0]<<8 | CanRxData.Data[1])>=8192u)) return;
#endif

    // 安排数据
    if (IS_MOTOR) {
        Motor_Type *motor = MOTOR;
        if(!motor) return; // Unbound feedback (e.g. 0x207) must not crash the control CPU.
        if(motor->type==MOTOR_TYPE_GM6020){
            Motor_Update(motor, CanRxData.Data, 3);
        }else{
            Motor_Update(motor, CanRxData.Data, 1);
        }
#if GIMBAL_TUNING_ENABLED
        if(type==CAN1_BRIDGE) GimbalTuning_MotorFeedback((uint16_t)deviceID);
#endif
    }else if(deviceID <= 0x188 && deviceID >= 0x181){
        deviceID = deviceID - 0x180 + 0x200 + 12;
        Motor_Type *motor = MOTOR;
        Motor_Update(motor, CanRxData.Data, 2);
    }
     else {
        for (i = 0; i < CanRxData.DLC; i++) {
            node              = CAN_NODE;
            node->isFirstByte = i == 0 ? 1 : 0;
            Protocol_Unpack(node, CanRxData.Data[i]);;
        }
    }
}

void Bridge_Send_Motor(Bridge_Type *bridge, uint8_t safetyMode) {
    static CAN_TypeDef *Canx[2]          = {CAN1, CAN2};
    static uint16_t     Can_Send_Id[4]   = {0x200, 0x1ff, 0x1fe, 0x2fe};
    static uint16_t     Can_ESC_Id[4][4] = {{0x201, 0x202, 0x203, 0x204}, {0x205, 0x206, 0x207, 0x208}, {0x205, 0x206, 0x207, 0x208}, {0x209, 0x020a, 0x20b, 0x20c}};
    static int          isNotEmpty       = 0; // 同一发送ID下是否有电机
    static Motor_Type * motor;                // 根据i,j,k锁定电机
    static int16_t      currents[4];          // CAN发送电流
    int                 i, j, k;              // CAN序号 发送ID序号 电调ID序号
    int                 motorEnabled;
    uint32_t            deviceID;
    uint8_t             type;
    uint8_t             forceFrame;
    static int16_t      Current_LK;             // LK电机速度控制
    static uint16_t     Can_Send_Id_LK  = 0x141;

    Bridge_Check_Motor_Watchdog(bridge); // 检查电机是否在线

    for (i = 0; i < 2; i++) {
        type = i == 0 ? CAN1_BRIDGE : CAN2_BRIDGE;
        //DJI
        for (j = 0; j < 4; j++) {
            isNotEmpty = 0;
            forceFrame = 0;
            for (k = 0; k < 4; k++) {
                deviceID     = Can_ESC_Id[j][k];
                motor        = MOTOR;
                motorEnabled = motor && motor->inputEnabled && motor->online && (motor->protocol == (j / 2) );
                currents[k]  = motorEnabled ? motor->input : 0;
#if GIMBAL_TUNING_ENABLED
                currents[k] = GimbalTuning_CANSlot((uint8_t)i,Can_Send_Id[j],(uint16_t)deviceID,currents[k],&forceFrame);
#endif
                isNotEmpty   = isNotEmpty || motorEnabled;
            }
            isNotEmpty = isNotEmpty || forceFrame;
#if GIMBAL_TUNING_ENABLED
            if(isNotEmpty && i==0 && Can_Send_Id[j]==0x1fe) {
                // The legacy Can_Send busy-waits forever on a disconnected bus.
                GimbalTuning_SendCAN(safetyMode?0:currents[0],safetyMode?0:currents[1],
                                    safetyMode?0:currents[2],safetyMode?0:currents[3]);
                continue;
            }
#endif
            if (isNotEmpty && !safetyMode) {
                Can_Send(Canx[i], Can_Send_Id[j], currents[0], currents[1], currents[2], currents[3]);
            } else if (isNotEmpty && safetyMode) {
                Can_Send(Canx[i], Can_Send_Id[j], 0, 0, 0, 0);
            }
        }
        //LK
        deviceID = Can_Send_Id_LK - 0x140 + 0x200 + 12;
        motor = MOTOR;
        motorEnabled = motor && motor->inputEnabled && motor->online;
        Current_LK = motorEnabled ? motor->input : 0;
        isNotEmpty = isNotEmpty || motorEnabled;
        if (isNotEmpty && !safetyMode) {
            Can_Send(Canx[i], Can_Send_Id_LK, 0xA100, 0, (Current_LK&0xff << 8) | (Current_LK >> 8), 0);
        } else if (isNotEmpty && safetyMode) {
            Can_Send(Canx[i], Can_Send_Id_LK, 0xA100, 0, 0, 0);
        }
    }
}

void Bridge_Check_Motor_Watchdog(Bridge_Type *bridge) {
    int         i;
    Motor_Type *motor;
    int8_t      vegtableMotorId = -1;
    for (i = 0; i < 26; i++) {
        motor = bridge->motors[i];
        //原写法超过65s后恒定判断电机下线
        if (motor != 0 && (uint16_t)(xTaskGetTickCount() - motor->updatedAt) > CAN_TIMEOUT) {
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