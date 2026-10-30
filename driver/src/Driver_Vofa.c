#include "Driver_Vofa.h"
#include "handle.h"

static uint8_t buffer[sizeof(debug_info_t)];

void Task_Vofa_Send(void * Parameters){
  TickType_t LastWakeTime = xTaskGetTickCount();
  ProtocolInfo_Type *protocolInfo = Parameters;
  Node_Type * node = protocolInfo->node;
  uint32_t deviceID = node->deviceID;
  while (1)
  {
    /* code */
    DMA_Disable(USARTx_Tx);
    DMA_Enable(USARTx_Tx, protocolInfo->length);
    vTaskDelayUntil(&LastWakeTime, 1000.0f/protocolInfo->frequency);
  }
  
}

void Vofa_Send(uint16_t frequency, uint16_t heapDeth, Node_Type * node, uint32_t commandID){
  ProtocolData.debugInfo.vofaData.vofaTail = vofaTail_t;
  ProtocolInfo_Type *protocolInfo = Protocol_Get_Info_Handle(commandID);
  if (protocolInfo->taskHandle != 0) {
    vTaskDelete(protocolInfo->taskHandle);
  }
  protocolInfo->node = node;
  protocolInfo->frequency = frequency;
  uint32_t deviceID = node->deviceID;
  BSP_DMA_Init(USARTx_Tx, ProtocolData.debugInfo.data, Protocol_Buffer_Length);
  xTaskCreate(Task_Vofa_Send, "Vofa_Send", heapDeth, (void *)protocolInfo, 6, protocolInfo->taskHandle);
};

/**
 * @brief Vofa调试初始化，修改Vofa_Debug_JustFloat_Type内float数组数量可以实现更多数据发送
 * @param deviceID USARTx x的值（USART3就填3）
 * @param vofa_data_num 要发送的数据个数，必须和Vofa_Debug_JustFloat_Type内data数据个数一致
 */
void Vofa_Debug_Init(Vofa_Debug_JustFloat_Type *vofa,uint32_t deviceID,uint8_t vofa_data_num){
  BSP_DMA_Init(USARTx_Tx,vofa->data,offsetof(Vofa_Debug_JustFloat_Type,data_num));
  vofa->data_num=vofa_data_num;
  vofa->tail[0] = 0x00;
  vofa->tail[1] = 0x00;
  vofa->tail[2] = 0x80;
  vofa->tail[3] = 0x7f;
}

/**
 * @brief Vofa发送数据
 * @param deviceID USARTx x的值（USART3就填3）
 */
void Vofa_Debug_Send(uint32_t deviceID){
    DMA_Disable(USARTx_Tx);
    DMA_Enable(USARTx_Tx,offsetof(Vofa_Debug_JustFloat_Type,data_num));
}