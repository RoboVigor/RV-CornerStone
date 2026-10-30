#include "handle.h"
#include "gimbal_tuning.h"
#include "Driver_GimbalTuning.h"
#include <string.h>

#if GIMBAL_TUNING_ENABLED
/* Single producer (UART ISR), single consumer (control task); no parsing in ISR. */
#define RX_CAPACITY 1024u
#define TX_CAPACITY 8u
#define RX_PER_TICK 128u
static volatile uint16_t rx_head,rx_tail;
static uint8_t rx_ring[RX_CAPACITY];
static uint32_t rx_arrival_us[RX_CAPACITY]; /* Liveness uses arrival, never delayed parsing time. */
static volatile uint32_t rx_errors,feedback_at_us[2],feedback_count[2];
static volatile uint8_t initialized;
static uint8_t tx_frames[TX_CAPACITY][GT_MAX_FRAME];
static uint16_t tx_lengths[TX_CAPACITY];
static uint8_t tx_head,tx_tail,tx_count,tx_active,tx_critical_loss;
static uint32_t observed_rx_errors;
static GT_Context tuning;
static GT_Feedback snapshot;
static uint8_t can_mailbox=0xffu,can_nonzero;
static uint32_t can_at_us;

static uint32_t time_us(void) { return TIM5->CNT; }

static void timer_init(void) {
    RCC_ClocksTypeDef clocks;
    uint32_t hz;
    RCC_GetClocksFreq(&clocks);
    hz=clocks.PCLK1_Frequency;
    if((RCC->CFGR & RCC_CFGR_PPRE1)!=0) hz*=2;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
    TIM5->CR1=0; TIM5->PSC=hz/1000000u-1u; TIM5->ARR=0xffffffffu;
    TIM5->EGR=TIM_EGR_UG; TIM5->CNT=0; TIM5->CR1=TIM_CR1_CEN;
}
static uint32_t boot_nonce(void) {
    uint32_t attempts=1000000u,value=0;
    /* No deterministic fallback: a repeated boot ID could accept stale commands. */
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG,ENABLE);
    RNG->CR=1u;
    while(attempts--) {
        if(RNG->SR & 6u) break; /* current clock/seed error */
        if(RNG->SR & 1u) { value=RNG->DR; if(value) break; }
    }
    RNG->CR=0;
    return value;
}
static int enqueue(void *user,const uint8_t *frame,uint16_t length) {
    (void)user;
    /* Reserve two entries for command/result/fault traffic, preserve FIFO order. */
    if(tx_count>=TX_CAPACITY || (frame[3]==GT_TELEMETRY && tx_count>=TX_CAPACITY-2)) {
        if(frame[3]!=GT_TELEMETRY) tx_critical_loss=1;
        return 0;
    }
    memcpy(tx_frames[tx_head],frame,length); tx_lengths[tx_head]=length;
    tx_head=(uint8_t)((tx_head+1u)%TX_CAPACITY); tx_count++;
    return 1;
}
static void tx_init(void) {
    DMA_InitTypeDef init;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
    DMA_Cmd(DMA1_Stream3,DISABLE);
    DMA_StructInit(&init);
    init.DMA_Channel=DMA_Channel_4;
    init.DMA_PeripheralBaseAddr=(uint32_t)&USART3->DR;
    init.DMA_Memory0BaseAddr=(uint32_t)tx_frames[0];
    init.DMA_DIR=DMA_DIR_MemoryToPeripheral;
    init.DMA_BufferSize=1;
    init.DMA_MemoryInc=DMA_MemoryInc_Enable;
    init.DMA_Mode=DMA_Mode_Normal;
    init.DMA_Priority=DMA_Priority_Medium;
    DMA_Init(DMA1_Stream3,&init);
    DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3|DMA_FLAG_HTIF3|DMA_FLAG_TEIF3|DMA_FLAG_DMEIF3|DMA_FLAG_FEIF3);
    USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
    /* Unlike the legacy helper, initialization does not enable/send garbage. */
}
static void tx_service(void) {
    if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TEIF3|DMA_FLAG_DMEIF3|DMA_FLAG_FEIF3)!=RESET) {
        DMA_Cmd(DMA1_Stream3,DISABLE);
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3|DMA_FLAG_HTIF3|DMA_FLAG_TEIF3|DMA_FLAG_DMEIF3|DMA_FLAG_FEIF3);
        tx_count=tx_head=tx_tail=tx_active=0;
        GT_TransportFault(&tuning);
        return;
    }
    if(tx_active) {
        if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)==RESET || (DMA1_Stream3->CR & DMA_SxCR_EN)) return;
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3|DMA_FLAG_HTIF3);
        tx_tail=(uint8_t)((tx_tail+1u)%TX_CAPACITY); tx_count--; tx_active=0;
    }
    if(tx_count && !(DMA1_Stream3->CR & DMA_SxCR_EN)) {
        DMA1_Stream3->M0AR=(uint32_t)tx_frames[tx_tail];
        DMA1_Stream3->NDTR=tx_lengths[tx_tail];
        DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3|DMA_FLAG_HTIF3);
        __DMB(); DMA_Cmd(DMA1_Stream3,ENABLE); tx_active=1;
    }
}
void GimbalTuning_Init(void) {
    timer_init();
    GT_Init(&tuning,boot_nonce(),enqueue,NULL);
    tx_init();
    Motor6020_MecArm_0_0.input=0; Motor6020_MecArm_0_1.input=0;
    initialized=1;
    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
    /* Revoke any pre-reset joint0 command before the 1.5s startup delay.
     * Slot 0x207 is unbound in this target; preserve existing joint3 command. */
    GimbalTuning_SendCAN(0,0,0,Motor6020_MecArm_3.input);
}
void GimbalTuning_SendCAN(int16_t slot0,int16_t slot1,int16_t slot2,int16_t slot3) {
    CanTxMsg frame;
    int16_t slots[4];
    uint8_t status; unsigned i; uint32_t now=time_us();
    tuning.now_us=now;
    if(!initialized || !GT_MotionAllowed(&tuning)) slot0=slot1=0;
    if(can_mailbox<3) {
        status=CAN_TransmitStatus(CAN1,can_mailbox);
        if(status==CAN_TxStatus_Ok) can_mailbox=0xffu;
        else if(status==CAN_TxStatus_Failed || now-can_at_us>=1000u ||
                (can_nonzero && slot0==0 && slot1==0)) {
            CAN_CancelTransmit(CAN1,can_mailbox); can_mailbox=0xffu;
            if(status==CAN_TxStatus_Failed || now-can_at_us>=1000u) {
                GT_TransportFault(&tuning); slot0=slot1=0;
                Motor6020_MecArm_0_0.input=Motor6020_MecArm_0_1.input=0;
            }
        } else return; /* At most one owned frame; never spin on the CAN bus. */
    }
    slots[0]=slot0; slots[1]=slot1; slots[2]=slot2; slots[3]=slot3;
    frame.StdId=0x1fe; frame.ExtId=0; frame.IDE=CAN_Id_Standard;
    frame.RTR=CAN_RTR_Data; frame.DLC=8;
    for(i=0;i<4;i++) { frame.Data[2*i]=(uint8_t)((uint16_t)slots[i]>>8); frame.Data[2*i+1]=(uint8_t)slots[i]; }
    can_mailbox=CAN_Transmit(CAN1,&frame); can_at_us=now; can_nonzero=(uint8_t)(slot0!=0 || slot1!=0);
    if(can_mailbox>=3) {
        GT_TransportFault(&tuning);
        Motor6020_MecArm_0_0.input=Motor6020_MecArm_0_1.input=0;
    }
}
void GimbalTuning_USART3_IRQHandler(void) {
    uint32_t sr=USART3->SR;
    if(sr & (USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)) {
        uint8_t byte=(uint8_t)USART3->DR; /* SR then DR also clears receive errors */
        uint16_t next=(uint16_t)((rx_head+1u)%RX_CAPACITY);
        if(sr & (USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)) rx_errors++;
        else if(initialized && (sr & USART_SR_RXNE)) {
            if(next==rx_tail) rx_errors++;
            else { rx_ring[rx_head]=byte; rx_arrival_us[rx_head]=time_us(); __DMB(); rx_head=next; }
        }
    }
}
void GimbalTuning_MotorFeedback(uint16_t id) {
    unsigned i;
    if(!initialized || (id!=0x205 && id!=0x206)) return;
    i=(unsigned)(id-0x205); feedback_at_us[i]=time_us(); feedback_count[i]++;
}
static void read_feedback(uint32_t now) {
    unsigned i;
    uint32_t mask=__get_PRIMASK();
    Motor_Type *motors[2]={&Motor6020_MecArm_0_0,&Motor6020_MecArm_0_1};
    Unitree_Motor_Type *other[2]={&Unitree_MecArm_1,&Unitree_MecArm_2};
    __disable_irq();
    for(i=0;i<2;i++) {
        snapshot.pos[i]=motors[i]->angle*(3.14159265358979323846f/180.0f);
        snapshot.vel[i]=(float)motors[i]->speed*(6.28318530717958647692f/60.0f);
        snapshot.current[i]=motors[i]->actualCurrent;
        snapshot.temperature[i]=(float)motors[i]->temperature;
        snapshot.online[i]=(uint8_t)(motors[i]->online && motors[i]->inputEnabled &&
            (motors[i]->direction==1 || motors[i]->direction==-1));
        snapshot.direction[i]=motors[i]->direction;
        snapshot.feedback_count[i]=feedback_count[i];
        snapshot.age_ms[i]=(now-feedback_at_us[i])/1000u;
        snapshot.other_pos[i]=other[i]->state.angle;
        snapshot.other_vel[i]=other[i]->state.speed;
        snapshot.other_online[i]=(uint8_t)(other[i]->state.online==1 && !other[i]->state.error_code);
        snapshot.other_age_ms[i]=(uint32_t)(xTaskGetTickCount()-other[i]->state.updated_at)*portTICK_PERIOD_MS;
    }
    snapshot.other_pos[2]=Motor6020_MecArm_3.angle*(3.14159265358979323846f/180.0f);
    snapshot.other_vel[2]=(float)Motor6020_MecArm_3.speed*(6.28318530717958647692f/60.0f);
    snapshot.other_online[2]=Motor6020_MecArm_3.online;
    snapshot.other_age_ms[2]=(uint16_t)(xTaskGetTickCount()-Motor6020_MecArm_3.updatedAt)*portTICK_PERIOD_MS;
    for(i=0;i<3;i++) snapshot.other_command[i]=MecArm.joint_target[i+1];
    snapshot.external_stop=SafetyMode;
    __set_PRIMASK(mask);
}
void GimbalTuning_Service(void) {
    unsigned count=0;
    uint32_t now=time_us(),errors;
    if(!initialized) return;
    read_feedback(now);
    tuning.feedback=snapshot; /* Current preconditions for CLEAR/START commands. */
    tuning.now_us=now;
    tx_service();
    errors=rx_errors;
    if(errors!=observed_rx_errors) {
        tuning.rx_dropped+=errors-observed_rx_errors; observed_rx_errors=errors;
        if(tuning.batch_active) GT_TransportFault(&tuning);
    }
    if(tx_critical_loss) { tx_critical_loss=0; GT_TransportFault(&tuning); }
    while(rx_tail!=rx_head && count++<RX_PER_TICK) {
        uint8_t byte; uint32_t arrival;
        __DMB(); byte=rx_ring[rx_tail]; arrival=rx_arrival_us[rx_tail];
        rx_tail=(uint16_t)((rx_tail+1u)%RX_CAPACITY);
        GT_Feed(&tuning,byte,arrival);
    }
    GT_Tick(&tuning,time_us(),&snapshot);
    if(GT_MotionAllowed(&tuning)) {
        Motor6020_MecArm_0_0.input=(int16_t)tuning.command[0];
        Motor6020_MecArm_0_1.input=(int16_t)tuning.command[1];
    } else { Motor6020_MecArm_0_0.input=0; Motor6020_MecArm_0_1.input=0; }
    tx_service();
}
int16_t GimbalTuning_CANSlot(uint8_t bus,uint16_t send_id,uint16_t id,int16_t original,uint8_t *force_frame) {
    if(bus!=0 || send_id!=0x1fe || (id!=0x205 && id!=0x206)) return original;
    *force_frame=1; /* Send explicit zeros even when BOTH feedback streams are offline. */
    if(!initialized || !GT_MotionAllowed(&tuning)) return 0;
    if(!Motor6020_MecArm_0_0.online || !Motor6020_MecArm_0_1.online ||
       !Motor6020_MecArm_0_0.inputEnabled || !Motor6020_MecArm_0_1.inputEnabled ||
       (time_us()-feedback_at_us[0])/1000u>tuning.safety.feedback_timeout_ms ||
       (time_us()-feedback_at_us[1])/1000u>tuning.safety.feedback_timeout_ms) return 0;
    return GT_CANSlot(&tuning,bus,send_id,id,original,force_frame);
}
#else
void GimbalTuning_Init(void) {}
void GimbalTuning_Service(void) {}
void GimbalTuning_USART3_IRQHandler(void) {}
void GimbalTuning_MotorFeedback(uint16_t id) { (void)id; }
void GimbalTuning_SendCAN(int16_t a,int16_t b,int16_t c,int16_t d) { (void)a; (void)b; (void)c; (void)d; }
int16_t GimbalTuning_CANSlot(uint8_t bus,uint16_t send_id,uint16_t id,int16_t original,uint8_t *force_frame) {
    (void)bus; (void)send_id; (void)id; (void)force_frame; return original;
}
#endif
