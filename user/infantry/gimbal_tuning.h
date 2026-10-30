#ifndef INFANTRY_GIMBAL_TUNING_H
#define INFANTRY_GIMBAL_TUNING_H
#include <stdint.h>

/* Tuning build owns USART3 and joint0 only. Legacy builds use config switch 0. */
void GimbalTuning_Init(void);
void GimbalTuning_Service(void);
void GimbalTuning_USART3_IRQHandler(void);
void GimbalTuning_MotorFeedback(uint16_t id);
/* Bounded sender for CAN1 0x1fe. Does not wait for an ESC ACK. */
void GimbalTuning_SendCAN(int16_t slot0,int16_t slot1,int16_t slot2,int16_t slot3);
int16_t GimbalTuning_CANSlot(uint8_t bus, uint16_t send_id, uint16_t motor_id,
                           int16_t original, uint8_t *force_frame);
#endif
