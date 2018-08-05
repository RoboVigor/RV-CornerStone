/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "led.h"
 

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
	extern int debug_tim;
	
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
 
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#include "Driver_DBUS.h"
#include "Driver_CAN.h"
#include "usart.h"
uint8_t UARTtemp;

/**
  * @brief  DBUS空闲中断(USART1)
  * @param  void
  * @retval void
  */
void USART1_IRQHandler(void)
{
    UARTtemp = USART1->DR;
    UARTtemp = USART1->SR;
    
    DMA_Cmd(DMA2_Stream2, DISABLE);
    
    //数据量正确
    if(DMA2_Stream2->NDTR == DBUSBackLength)
    {
      //  DBUSFrameCounter++;         //帧数增加
        DBUS_DataDecoding();          //解码
    }
    
    //重启DMA
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
    while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
    DMA_SetCurrDataCounter(DMA2_Stream2, DBUSLength + DBUSBackLength);
    DMA_Cmd(DMA2_Stream2, ENABLE);
}


/**
  * @brief  1000Hz任务循环中断
  * @param  void
  * @retval void
  */
void TIM6_DAC_IRQHandler(void)  
{
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET) 
	  {
			TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
      TIM_ClearFlag(TIM6, TIM_FLAG_Update);
			
			//Set_CM_Speed(CAN1,1000,0,0,0);
    }
}

CanRxMsg CanRxData;
/**
  * @brief  CAN数据接收中断服务函数
  * @param  void
  * @retval void
  */
void CAN1_RX0_IRQHandler(void)
{
		CAN_Receive(CAN1, CAN_FIFO0, &CanRxData);
		switch(CanRxData.StdId)
		{
			case WHEEL_1_ID :
				Motor_Feedback.Motor_201_Agree = (short)( (int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
				Motor_Feedback.Motor_201_Speed = (short)( (int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
			break;
			
			case WHEEL_2_ID :
				Motor_Feedback.Motor_202_Agree = (short)( (int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
				Motor_Feedback.Motor_202_Speed = (short)( (int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
			break;
			
			case WHEEL_3_ID :
				Motor_Feedback.Motor_203_Agree = (short)( (int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
				Motor_Feedback.Motor_203_Speed = (short)( (int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
			break;
			
			case WHEEL_4_ID :
				Motor_Feedback.Motor_204_Agree = (short)( (int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
				Motor_Feedback.Motor_204_Speed = (short)( (int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
			break;
			
			
			
			
			default:
				break;
			
		}
				
				
			
}

void CAN2_RX0_IRQHandler(void)
{
	CAN_Receive(CAN2, CAN_FIFO0, &CanRxData);
	//printf("%d\r\n",CanRxData.StdId);
		switch(CanRxData.StdId)
		{
			//RED_LIGHT_ON;
			case 0x201://钩子电机
				Motor_Feedback.Motor_205_Agree = (short)( (int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
				Motor_Feedback.Motor_205_Speed = (short)( (int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
			break;
			
			case 0x202:
				
				Motor_Feedback.Motor_206_Agree = (short)( (int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
				Motor_Feedback.Motor_206_Speed = (short)( (int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
			
			break;
			
			case 0x203:
				Motor_Feedback.Motor_207_Agree = (short)( (int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
				Motor_Feedback.Motor_207_Speed = (short)( (int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
					
				
			
			break;
			
			default:
				break;
			
		}
}
	   void TIM2_IRQHandler(void)
	{
		if(TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET)
		{
			//LED2=!LED2;//视频中的方式
			//RED_LIGHT_TOGGLE;
			debug_tim=debug_tim+100;
			TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		}
	}

