/* Host-only register model for logic tests of the actual STM32 adapter.
 * This is NOT vendor API/ABI validation; see the separate real-header ARM check. */
#ifndef GIMBAL_PLATFORM_RUNTIME_H
#define GIMBAL_PLATFORM_RUNTIME_H
#define __HANDLE_H
#define GIMBAL_TUNING_ENABLED 1
#include <stdint.h>
#include <string.h>
#define portTICK_PERIOD_MS 1u
#define ENABLE 1
#define DISABLE 0
#define RESET 0
#define RCC_CFGR_PPRE1 0x1c00u
#define RCC_APB1Periph_TIM5 1u
#define RCC_AHB2Periph_RNG 2u
#define RCC_AHB1Periph_DMA1 3u
#define TIM_EGR_UG 1u
#define TIM_CR1_CEN 1u
#define DMA_SxCR_EN 1u
#define DMA_Channel_4 4u
#define DMA_DIR_MemoryToPeripheral 1u
#define DMA_MemoryInc_Enable 1u
#define DMA_Mode_Normal 0u
#define DMA_Priority_Medium 1u
#define USART_DMAReq_Tx 1u
#define USART_IT_RXNE 1u
#define USART_SR_RXNE 0x20u
#define USART_SR_ORE 8u
#define USART_SR_NE 4u
#define USART_SR_FE 2u
#define USART_SR_PE 1u
#define DMA_FLAG_TCIF3 1u
#define DMA_FLAG_HTIF3 2u
#define DMA_FLAG_TEIF3 4u
#define DMA_FLAG_DMEIF3 8u
#define DMA_FLAG_FEIF3 16u
#define CAN_TxStatus_Ok 1u
#define CAN_TxStatus_Failed 0u
#define CAN_TxStatus_Pending 2u
#define CAN_Id_Standard 0u
#define CAN_RTR_Data 0u
#define __DMB() ((void)0)
#define __disable_irq() ((void)0)
#define __get_PRIMASK() 0u
#define __set_PRIMASK(v) ((void)(v))
typedef struct { uint32_t CNT,CR1,PSC,ARR,EGR; } TIM_TypeDef;
typedef struct { uint32_t CFGR; } RCC_TypeDef;
typedef struct { uint32_t CR,SR,DR; } RNG_TypeDef;
typedef struct { uint32_t SR,DR; } USART_TypeDef;
typedef struct { uint32_t CR,M0AR,NDTR; } DMA_Stream_TypeDef;
typedef struct { uint32_t PCLK1_Frequency; } RCC_ClocksTypeDef;
typedef struct { uint32_t DMA_Channel,DMA_PeripheralBaseAddr,DMA_Memory0BaseAddr,DMA_DIR,DMA_BufferSize,DMA_MemoryInc,DMA_Mode,DMA_Priority; } DMA_InitTypeDef;
typedef struct { uint32_t StdId,ExtId; uint8_t IDE,RTR,DLC,Data[8]; } CanTxMsg;
typedef struct { uint8_t unused; } CAN_TypeDef;
extern TIM_TypeDef fake_tim;
extern RCC_TypeDef fake_rcc;
extern RNG_TypeDef fake_rng;
extern USART_TypeDef fake_uart;
extern DMA_Stream_TypeDef fake_dma;
extern CAN_TypeDef fake_can;
#define TIM5 (&fake_tim)
#define RCC (&fake_rcc)
#define RNG (&fake_rng)
#define USART3 (&fake_uart)
#define DMA1_Stream3 (&fake_dma)
#define CAN1 (&fake_can)
uint32_t xTaskGetTickCount(void);
void RCC_GetClocksFreq(RCC_ClocksTypeDef *p);
void RCC_APB1PeriphClockCmd(uint32_t p,int enable);
void RCC_AHB2PeriphClockCmd(uint32_t p,int enable);
void RCC_AHB1PeriphClockCmd(uint32_t p,int enable);
void DMA_StructInit(DMA_InitTypeDef *p);
void DMA_Init(DMA_Stream_TypeDef *d,DMA_InitTypeDef *p);
void DMA_Cmd(DMA_Stream_TypeDef *d,int enable);
void DMA_ClearFlag(DMA_Stream_TypeDef *d,uint32_t flags);
int DMA_GetFlagStatus(DMA_Stream_TypeDef *d,uint32_t flags);
void USART_DMACmd(USART_TypeDef *p,uint32_t req,int enable);
void USART_ITConfig(USART_TypeDef *p,uint32_t req,int enable);
uint8_t CAN_Transmit(CAN_TypeDef *can,CanTxMsg *p);
uint8_t CAN_TransmitStatus(CAN_TypeDef *can,uint8_t box);
void CAN_CancelTransmit(CAN_TypeDef *can,uint8_t box);
typedef struct { float angle,actualCurrent; int16_t speed,temperature,input; uint16_t updatedAt; uint8_t online; int8_t inputEnabled,direction; } Motor_Type;
typedef struct { struct { float angle,speed; uint8_t online,error_code; uint32_t updated_at; } state; } Unitree_Motor_Type;
extern Motor_Type Motor6020_MecArm_0_0,Motor6020_MecArm_0_1,Motor6020_MecArm_3;
extern Unitree_Motor_Type Unitree_MecArm_1,Unitree_MecArm_2;
extern uint8_t SafetyMode;
typedef struct { float joint_target[5]; } TestArm;
extern TestArm MecArm;
#endif
