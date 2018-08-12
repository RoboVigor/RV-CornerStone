#ifndef __DRIVER_CAN_H
#define __DRIVER_CAN_H

#include "stm32f4xx.h"

#define WHEEL_1_ID                  0x201
#define WHEEL_2_ID                  0x202
#define WHEEL_3_ID                  0x203
#define WHEEL_4_ID                  0x204

#define RATE_BUF_SIZE               6

typedef struct 
{
	uint16_t Motor_201_Agree;  
	int16_t  Motor_201_Speed;					
	
	uint16_t Motor_202_Agree;
	int16_t  Motor_202_Speed;
	
	uint16_t Motor_203_Agree;
	int16_t  Motor_203_Speed;
	
	uint16_t Motor_204_Agree;
	int16_t  Motor_204_Speed;
	
	uint16_t Motor_205_Agree;//钩子
	int16_t Motor_205_Speed;
	
	uint16_t Motor_206_Agree; //armour的第一个电机 
	int16_t  Motor_206_Speed;
	
	uint16_t Motor_207_Agree;  //armor的第二个电机
	int16_t  Motor_207_Speed;
	

}Motor_Feedback_Structure;

typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	//buf,for filter
	int32_t round_cnt;										//圈数
	int32_t filter_rate;											//速度
	float ecd_angle;											//角度
}Encoder;

#ifdef  __DRIVER_CAN_GLOBALS
#define __DRIVER_CAN_EXT
#else
#define __DRIVER_CAN_EXT extern
#endif

__DRIVER_CAN_EXT Motor_Feedback_Structure Motor_Feedback;
__DRIVER_CAN_EXT volatile Encoder Hook_Encoder,Armour1_Encoder,Armour2_Encoder;

void Set_CM_Speed(CAN_TypeDef *CANx, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);
void GetEncoderBias(volatile Encoder *v);
void EncoderProcess(volatile Encoder *v,uint16_t agree);
void Set_Hook_Armour_Speed(CAN_TypeDef *CANx, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204);//201(205)是hook的 202(206) 203(207)是armour的

#endif

