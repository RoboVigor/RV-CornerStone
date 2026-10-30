#include "stubs/gimbal_platform_runtime.h"
#include <assert.h>
#include <stdio.h>
/* Include the real adapter in this translation unit so its private bounded
 * queues/register decisions can be inspected without adding firmware test APIs. */
#include "../user/infantry/gimbal_tuning.c"

TIM_TypeDef fake_tim;
RCC_TypeDef fake_rcc;
RNG_TypeDef fake_rng;
USART_TypeDef fake_uart;
DMA_Stream_TypeDef fake_dma;
CAN_TypeDef fake_can;
Motor_Type Motor6020_MecArm_0_0,Motor6020_MecArm_0_1,Motor6020_MecArm_3;
Unitree_Motor_Type Unitree_MecArm_1,Unitree_MecArm_2;
uint8_t SafetyMode;
TestArm MecArm;
static uint32_t dma_flags;
static unsigned can_sends,can_cancels,dma_starts;
static uint8_t can_status=CAN_TxStatus_Ok,can_return;
static CanTxMsg last_can;
uint32_t xTaskGetTickCount(void) { return fake_tim.CNT/1000u; }
void RCC_GetClocksFreq(RCC_ClocksTypeDef *p) { p->PCLK1_Frequency=45000000; }
void RCC_APB1PeriphClockCmd(uint32_t p,int e) { (void)p;(void)e; }
void RCC_AHB2PeriphClockCmd(uint32_t p,int e) { (void)p;(void)e; }
void RCC_AHB1PeriphClockCmd(uint32_t p,int e) { (void)p;(void)e; }
void DMA_StructInit(DMA_InitTypeDef *p) { memset(p,0,sizeof *p); }
void DMA_Init(DMA_Stream_TypeDef *d,DMA_InitTypeDef *p) { d->M0AR=p->DMA_Memory0BaseAddr; d->NDTR=p->DMA_BufferSize; }
void DMA_Cmd(DMA_Stream_TypeDef *d,int e) { if(e) { d->CR|=DMA_SxCR_EN; dma_starts++; } else d->CR&=~DMA_SxCR_EN; }
void DMA_ClearFlag(DMA_Stream_TypeDef *d,uint32_t f) { (void)d; dma_flags&=~f; }
int DMA_GetFlagStatus(DMA_Stream_TypeDef *d,uint32_t f) { (void)d; return (dma_flags&f)!=0; }
void USART_DMACmd(USART_TypeDef *p,uint32_t r,int e) { (void)p;(void)r;(void)e; }
void USART_ITConfig(USART_TypeDef *p,uint32_t r,int e) { (void)p;(void)r;(void)e; }
uint8_t CAN_Transmit(CAN_TypeDef *can,CanTxMsg *p) { (void)can; last_can=*p; can_sends++; return can_return; }
uint8_t CAN_TransmitStatus(CAN_TypeDef *can,uint8_t box) { (void)can;(void)box; return can_status; }
void CAN_CancelTransmit(CAN_TypeDef *can,uint8_t box) { (void)can;(void)box; can_cancels++; }
static int16_t slot(unsigned i) { return (int16_t)((uint16_t)last_can.Data[2*i]<<8|last_can.Data[2*i+1]); }
static void reset(void) {
    GT_Init(&tuning,123,enqueue,NULL); tuning.session=1;
    rx_head=rx_tail=0; rx_errors=observed_rx_errors=0;
    tx_head=tx_tail=tx_count=tx_active=tx_critical_loss=0;
    dma_flags=0; fake_dma.CR=0; can_mailbox=0xff; can_nonzero=0;
    fake_tim.CNT=0; can_sends=can_cancels=dma_starts=0;
    can_return=0; can_status=CAN_TxStatus_Ok; initialized=1;
}
static void arm(void) {
    tuning.state=GT_RUN; tuning.batch_active=1; tuning.motion_enabled=1;
    tuning.session=1; tuning.command[0]=111; tuning.command[1]=222;
}
static void test_boot_and_can(void) {
    reset(); fake_rng.SR=1; fake_rng.DR=12345; fake_rcc.CFGR=0x1000;
    Motor6020_MecArm_3.input=-456;
    GimbalTuning_Init();
    assert(tuning.boot_id==12345 && fake_tim.PSC==89 && can_sends==1);
    assert(slot(0)==0 && slot(1)==0 && slot(2)==0 && slot(3)==-456);
    assert(dma_starts==0); /* TX init must not send an uninitialized buffer. */
    arm(); fake_tim.CNT=100;
    GimbalTuning_SendCAN(111,222,345,-456);
    assert(can_sends==2 && slot(0)==111 && slot(1)==222 && slot(2)==345 && slot(3)==-456);
    can_status=CAN_TxStatus_Pending; fake_tim.CNT=500;
    GimbalTuning_SendCAN(111,222,345,-456); assert(can_sends==2);
    tuning.motion_enabled=0; tuning.batch_active=0; fake_tim.CNT=600;
    GimbalTuning_SendCAN(0,0,346,-457);
    assert(can_cancels==1 && slot(0)==0 && slot(1)==0 && slot(2)==346 && slot(3)==-457 && !tuning.fault);
    can_status=CAN_TxStatus_Ok; fake_tim.CNT=700; arm(); GimbalTuning_SendCAN(111,222,347,-458);
    can_status=CAN_TxStatus_Pending; fake_tim.CNT=1800;
    GimbalTuning_SendCAN(111,222,348,-459);
    assert(tuning.fault==GT_F_TRANSPORT && slot(0)==0 && slot(1)==0 && slot(2)==348 && slot(3)==-459);
    assert(!GT_MotionAllowed(&tuning));
    reset(); arm(); can_return=4;
    GimbalTuning_SendCAN(111,222,1,2); assert(tuning.fault==GT_F_TRANSPORT && can_sends==1);
}
static void test_dma_queue(void) {
    uint8_t frame[276]; uint16_t n;
    reset(); n=GT_Encode(frame,GT_STATUS,123,1,1,NULL,0);
    assert(enqueue(NULL,frame,n)); tx_service(); assert(tx_active && dma_starts==1 && tx_count==1);
    assert(enqueue(NULL,frame,n)); tx_service(); assert(dma_starts==1 && tx_count==2);
    dma_flags=DMA_FLAG_TCIF3; fake_dma.CR=0; tx_service();
    assert(dma_starts==2 && tx_count==1 && tx_active);
    dma_flags=DMA_FLAG_TEIF3; tx_service(); assert(tuning.fault==GT_F_TRANSPORT);
}
static void uart_bytes(const uint8_t *p,unsigned n,uint32_t arrival) {
    unsigned i; fake_tim.CNT=arrival;
    for(i=0;i<n;i++) { fake_uart.SR=USART_SR_RXNE; fake_uart.DR=p[i]; GimbalTuning_USART3_IRQHandler(); }
}
static void test_arrival_and_overflow(void) {
    uint8_t frame[276]; uint16_t n; unsigned i;
    reset(); tuning.safety.heartbeat_timeout_ms=100; tuning.safety.telemetry_period_ms=10;
    tuning.safety_valid=1; tuning.heartbeat_valid=1; tuning.heartbeat_at_us=0; arm();
    n=GT_Encode(frame,GT_HEARTBEAT,123,1,1,NULL,0); uart_bytes(frame,n,1000);
    assert(rx_arrival_us[0]==1000 && tuning.last_seq==0); /* ISR never parses. */
    fake_tim.CNT=500000; GimbalTuning_Service();
    assert(tuning.heartbeat_at_us==1000 && tuning.fault==GT_F_HEARTBEAT);
    reset(); arm();
    for(i=0;i<RX_CAPACITY+8;i++) { fake_uart.SR=USART_SR_RXNE; fake_uart.DR=0; GimbalTuning_USART3_IRQHandler(); }
    assert(rx_errors>0); fake_tim.CNT=1000; GimbalTuning_Service();
    assert(tuning.fault==GT_F_TRANSPORT && tuning.rx_dropped>0);
    assert(rx_tail==RX_PER_TICK); /* bounded drain, no unbounded parser loop */
}
int main(void) {
    test_boot_and_can(); test_dma_queue(); test_arrival_and_overflow();
    puts("gimbal adapter: startup zero, bounded CAN, preserved slots, DMA ownership and ISR arrival passed");
    return 0;
}
