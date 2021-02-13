#include "Driver_Bridge.h"

void Bridge_Bind(Bridge_Type *bridge, uint8_t type, uint32_t deviceID, void *handle) {
    if (IS_MOTOR) {
        MOTOR = handle;
    } else {
        Protocol_Channel_Type *channel = handle;
        if (IS_CAN) {
            CAN_CHANNEL = channel;
        } else {
            BSP_DMA_Init(USART[deviceID].Rx, channel->sendBuf, Protocol_Buffer_Length);
            BSP_DMA_Init(USART[deviceID].Tx, channel->receiveBuf, Protocol_Buffer_Length);
            USART_CHANNEL = channel;
        }
        channel->deviceID   = deviceID;
        channel->bridgeType = type;
    }
}

void Bridge_Receive_USART(Bridge_Type *bridge, uint8_t type, uint32_t deviceID) {
    uint16_t               i, tmp, len;
    Protocol_Channel_Type *channel = USART_CHANNEL;

    // clear IDLE flag
    tmp = USARTx->DR;
    tmp = USARTx->SR;

    // disabe DMA
    DMA_Disable(USARTx_Rx);

    // unpack
    len = Protocol_Buffer_Length - DMA_Get_Data_Counter(USARTx_Rx);
    for (i = 0; i < len; i++) {
        Protocol_Unpack(channel, channel->receiveBuf[i]);
    }

    // enable DMA
    DMA_Enable(USARTx_Rx, Protocol_Buffer_Length);
}

void Bridge_Receive_CAN(Bridge_Type *bridge, uint8_t type) {
    uint16_t               i;
    Protocol_Channel_Type *channel;
    uint32_t               deviceID;
    CanRxMsg               CanRxData;

    // 读取数据
    CAN_Receive(type == CAN1_BRIDGE ? CAN1 : CAN2, CAN_FIFO0, &CanRxData);
    deviceID = CanRxData.StdId;

    // 安排数据
    if (IS_MOTOR) {
        Motor_Update(MOTOR, CanRxData.Data);
    } else {
        for (i = 0; i < CanRxData.DLC; i++) {
            channel = CAN_CHANNEL;
            Protocol_Unpack(channel, CanRxData.Data[i]);
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
    uint32_t            deviceID;
    uint8_t             type;

    for (i = 0; i < 2; i++) {
        type = i == 0 ? CAN1_BRIDGE : CAN2_BRIDGE;
        for (j = 0; j < 3; j++) {
            isNotEmpty = 0;
            for (k = 0; k < 4; k++) {
                deviceID    = Can_ESC_Id[j][k];
                motor       = MOTOR;
                currents[k] = (motor && motor->inputEnabled) ? motor->input : 0;
                isNotEmpty  = isNotEmpty || (motor && motor->inputEnabled);
            }
            if (isNotEmpty && !safetyMode) {
                Can_Send(Canx[i], Can_Send_Id[j], currents[0], currents[1], currents[2], currents[3]);
            } else if (isNotEmpty && safetyMode) {
                Can_Send(Canx[i], Can_Send_Id[j], 0, 0, 0, 0);
            }
        }
    }
}

void Bridge_Send_Protocol(Bridge_Type *bridge, Protocol_Channel_Type *channel, uint32_t commandID) {
    uint32_t deviceID = channel->deviceID;
    uint16_t dataLength;
    uint8_t  type = channel->bridgeType;

    if (IS_CAN) {
        dataLength = Protocol_Pack(channel, commandID);
        Can_Send_Msg(type == CAN1_BRIDGE ? CAN1 : CAN2, CAN_DEVICE_ID, channel->sendBuf, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
    } else {
        DMA_Disable(USARTx_Tx);
        dataLength = Protocol_Pack(channel, commandID);
        DMA_Enable(USARTx_Tx, PROTOCOL_HEADER_CRC_CMDID_LEN + dataLength);
    }
}
