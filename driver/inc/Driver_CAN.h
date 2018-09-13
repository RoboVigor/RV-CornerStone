#ifndef __DRIVER_CAN_H
#define __DRIVER_CAN_H

#include "stm32f4xx.h"

#define WHEEL_1_ID 0x201
#define WHEEL_2_ID 0x202
#define WHEEL_3_ID 0x203
#define WHEEL_4_ID 0x204

#define RATE_BUF_SIZE 6

typedef struct {
    uint16_t motor201Angle;
    int16_t  motor201Speed;

    uint16_t motor202Angle;
    int16_t  motor202Speed;

    uint16_t motor203Angle;
    int16_t  motor203Speed;

    uint16_t motor204Angle;
    int16_t  motor204Speed;

    uint16_t motor205Angle; //钩子
    int16_t  motor205Speed;

    uint16_t motor206Angle; // armour的第一个电机
    int16_t  motor206Speed;

    uint16_t motor207Angle; // armor的第二个电机
    int16_t  motor207Speed;

} MotorFeedback_Type;

typedef struct {
    int32_t rawValue;               //编码器不经处理的原始值
    int32_t lastRawValue;           //上一次的编码器原始值
    int32_t ecdValue;               //经过处理后连续的编码器值
    int32_t diff;                   //两次编码器之间的差值
    int32_t tempCount;              //计数用
    uint8_t bufCount;               //滤波更新buf用
    int32_t ecdBias;                //初始编码器值
    int32_t ecdRawRate;             //通过编码器计算得到的速度原始值
    int32_t rateBuf[RATE_BUF_SIZE]; // buf,for filter
    int32_t roundCnt;               //圈数
    int32_t filterRate;             //速度
    float   ecdAngle;               //角度
} CANEncoder_Type;

void Can_Set_CM_Current(CAN_TypeDef *CANx, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);
void CAN_Get_Encoder_Bias(volatile CANEncoder_Type *v);
void CAN_Update_Encoder_Data(volatile CANEncoder_Type *v, uint16_t agree);
void CAN_Set_HookArmour_Speed(CAN_TypeDef *CANx, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204); // 201(205)是hook的 202(206) 203(207)是armour的

#endif
