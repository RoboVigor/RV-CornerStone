#define __DRIVER_CAN_GLOBALS
#include "Driver_CAN.h"

/**
  * @brief  发送数据给四个电机
  * @param  CANx  指定一个CAN口,CAN1或CAN2
	*					i_201 0x201地址电机给定电流值，范围-32768~32768
	*					i_202 0x202地址电机给定电流值，范围-32768~32768
	*					i_203 0x203地址电机给定电流值，范围-32768~32768
	*					i_204 0x204地址电机给定电流值，范围-32768~32768
  * @retval void
  */
	void Set_CM_Speed(CAN_TypeDef *CANx, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE   = CAN_Id_Standard;
    tx_message.RTR   = CAN_RTR_Data;
    tx_message.DLC   = 0x08;
    
    tx_message.Data[0] = (uint8_t)(i_201 >> 8);
    tx_message.Data[1] = (uint8_t)i_201;
    tx_message.Data[2] = (uint8_t)(i_202 >> 8);
    tx_message.Data[3] = (uint8_t)i_202;
    tx_message.Data[4] = (uint8_t)(i_203 >> 8);
    tx_message.Data[5] = (uint8_t)i_203;
    tx_message.Data[6] = (uint8_t)(i_204 >> 8);
    tx_message.Data[7] = (uint8_t)i_204;
		
	  //CAN故障重启
		do
		{
			if(CAN1->ESR)
			{
				CAN1->MCR |= 0x02;
				CAN1->MCR &= 0xFD;
			}
		}while(!(CAN1->TSR & 0x1C000000));
		
    CAN_Transmit(CANx,&tx_message);
}



void Set_Hook_Armour_Speed(CAN_TypeDef *CANx, int16_t i_201, int16_t i_202, int16_t i_203, int16_t i_204)
{
		
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE   = CAN_Id_Standard;
    tx_message.RTR   = CAN_RTR_Data;
    tx_message.DLC   = 0x08;

    tx_message.Data[0] = (uint8_t)(i_201 >> 8);
    tx_message.Data[1] = (uint8_t)i_201;
    tx_message.Data[2] = (uint8_t)(i_202 >> 8);
    tx_message.Data[3] = (uint8_t)i_202;
    tx_message.Data[4] = (uint8_t)(i_203 >> 8);
    tx_message.Data[5] = (uint8_t)i_203;
    tx_message.Data[6] = (uint8_t)(i_204 >> 8);
    tx_message.Data[7] = (uint8_t)i_204;
	
		
	  //CAN故障重启
		do
		{
			if(CAN2->ESR)
			{
				CAN2->MCR |= 0x02;
				CAN2->MCR &= 0xFD;
			}
		}while(!(CAN2->TSR & 0x1C000000));
		
    CAN_Transmit(CANx,&tx_message);
}



/**
  * @brief  获取编码器的值
  * @param  void
  * @retval void
  */
void GetEncoderBias(volatile Encoder *v)
{
//		int i;
	//v->ecd_bias = 4333;  //保存初始编码器值作为偏差  
    v->raw_value = v->ecd_bias;
    v->last_raw_value = v->ecd_bias;
		v->round_cnt=0;
}

/**
  * @brief  电机机械角度解结解算，转子角度转成实际角度
  * @param  void
  * @retval void
  */


 void EncoderProcess(volatile Encoder *v,uint16_t agree)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = agree;
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4200)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4200)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);					
}



