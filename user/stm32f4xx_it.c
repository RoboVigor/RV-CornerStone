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
#include "main.h"

// extern int debugTime;

int ArmatureRotateSpeed[4], Buffer[4];
int isDoublePID = 0; // 0为单速度pid

float LastYawAngleFeed = 0;
float YawAngleFeedOffset = 0;
float YawAngleFeedThreshold = 0.003;
float YawAngleFeedDiff = 0;
float YawAngleFeedOffsetSample = 0;
float YawAngleFeedOffsetSampleCounter = 0;
int PanPIDMode = 1;

int32_t debug_YawAngleFeed = 0;
int32_t debug_value1 = 0;
int32_t debug_value2 = 0;
int32_t debug_value3 = 0;
int32_t debug_value4 = 0;
int32_t debug_value5 = 0;
int32_t debug_value6 = 0;
int32_t debug_value7 = 0;
int32_t debug_value8 = 0;

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

void NMI_Handler(void) {}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void) {
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1) {
  }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void) {
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1) {
  }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void) {
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1) {
  }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void) {
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1) {
  }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
// void SVC_Handler(void)
//{
//}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void) {}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
// void PendSV_Handler(void)
//{
//}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
// void SysTick_Handler(void)
//{
//
//}

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
#include "Driver_CAN.h"
#include "Driver_DBUS.h"
#include "usart.h"
uint8_t UARTtemp;

/**
 * @brief  DBUS空闲中断(USART1)
 * @param  void
 * @retval void
 */
void USART1_IRQHandler(void) {

}

/**
 * @brief  无线串口中断(USART3)
 * @param  void
 * @retval void
 */
void USART3_IRQHandler(void) {
    vTaskResume(TaskHandler_USART3);
}

/**
 * @brief  1000Hz任务循环中断
 * @param  void
 * @retval void
 */
void TIM6_DAC_IRQHandler(void) {
  if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);

    // Set_CM_Speed(CAN1,1000,0,0,0);
  }
}

CanRxMsg CanRxData;
/**
 * @brief  CAN数据接收中断服务函数
 * @param  void
 * @retval void
 */
void CAN1_RX0_IRQHandler(void) {
  CAN_Receive(CAN1, CAN_FIFO0, &CanRxData);
  switch (CanRxData.StdId) {
  case WHEEL_1_ID:
    Motor_Feedback.Motor_201_Agree =
        (short)((int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    Motor_Feedback.Motor_201_Speed =
        (short)((int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
    break;

  case WHEEL_2_ID:
    Motor_Feedback.Motor_202_Agree =
        (short)((int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    Motor_Feedback.Motor_202_Speed =
        (short)((int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
    break;

  case WHEEL_3_ID:
    Motor_Feedback.Motor_203_Agree =
        (short)((int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    Motor_Feedback.Motor_203_Speed =
        (short)((int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
    break;

  case WHEEL_4_ID:
    Motor_Feedback.Motor_204_Agree =
        (short)((int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    Motor_Feedback.Motor_204_Speed =
        (short)((int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
    break;

  default:
    break;
  }
}

void CAN2_RX0_IRQHandler(void) {
  CAN_Receive(CAN2, CAN_FIFO0, &CanRxData);
  // printf("%d\r\n",CanRxData.StdId);
  switch (CanRxData.StdId) {
  // RED_LIGHT_ON;
  case 0x201: //钩子电机
    Motor_Feedback.Motor_205_Agree =
        (short)((int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    Motor_Feedback.Motor_205_Speed =
        (short)((int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);
    break;

  case 0x202:

    Motor_Feedback.Motor_206_Agree =
        (short)((int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    Motor_Feedback.Motor_206_Speed =
        (short)((int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);

    break;

  case 0x203:
    Motor_Feedback.Motor_207_Agree =
        (short)((int)CanRxData.Data[0] << 8 | CanRxData.Data[1]);
    Motor_Feedback.Motor_207_Speed =
        (short)((int)CanRxData.Data[2] << 8 | CanRxData.Data[3]);

    break;

  default:
    break;
  }
}

void mainTask(void) {

  if (DBUS_ReceiveData.switch_right == 2) {
    Set_CM_Speed(CAN1, 0, 0, 0, 0);
    return;
  }
  Motion_Update();
  if (ABS(DBUS_ReceiveData.ch1) < 5) {
    PanPIDMode = 2;
  } else {
    PanPIDMode = 1;
  }

  if (Euler_Angle.Yaw - YawAngleFeedOffset - LastYawAngleFeed > 300) {
    YawAngleFeedOffset += 360;
  } else if (Euler_Angle.Yaw - YawAngleFeedOffset - LastYawAngleFeed < -300) {
    YawAngleFeedOffset -= 360;
  }

  YawAngleFeed = Euler_Angle.Yaw - YawAngleFeedOffset;

  YawAngleFeedDiff = YawAngleFeed - LastYawAngleFeed;

  if (ABS(DBUS_ReceiveData.ch1) < 10 && ABS(DBUS_ReceiveData.ch3) < 10 &&
      ABS(DBUS_ReceiveData.ch4) < 10)
  // if(1)
  {
    if (ABS(YawAngleFeedDiff) < YawAngleFeedThreshold) {
      YawAngleFeedOffset += YawAngleFeedDiff;
      YawAngleFeed = LastYawAngleFeed;
      PanPIDMode = 0;
      if (YawAngleFeedOffsetSampleCounter < 100) {
        YawAngleFeedOffsetSample += YawAngleFeedDiff;
        YawAngleFeedOffsetSampleCounter += 1;
      }
    } else {
      LastYawAngleFeed = YawAngleFeed;
    }
  } else {
    if (DBUS_ReceiveData.switch_right == 1) {
      YawAngleFeedOffset +=
          YawAngleFeedOffsetSample / YawAngleFeedOffsetSampleCounter;
    }
    YawAngleFeed = Euler_Angle.Yaw - YawAngleFeedOffset;
    YawAngleFeedDiff = YawAngleFeed - LastYawAngleFeed;
    LastYawAngleFeed = YawAngleFeed;
  }
  /*DEBUG*/
  debug_YawAngleFeed = (int)(YawAngleFeed);
  debug_value1 = (int)(DBUS_ReceiveData.ch1);
  debug_value2 = (int)(DBUS_ReceiveData.ch2);
  debug_value3 = (int)(DBUS_ReceiveData.ch3);
  debug_value4 = (int)(DBUS_ReceiveData.ch4);
  debug_value5 = (int)(YawAngleFeedOffsetSample);
  debug_value6 = (int)(YawAngleFeedOffsetSampleCounter);
  debug_value7 = (int)(YawAngleFeedOffset * 10000);
  debug_value8 =
      (int)(YawAngleFeedOffsetSample / YawAngleFeedOffsetSampleCounter * 10000);

  if (DBUS_ReceiveData.switch_left == 1) //摄像头朝向丝杆 功能：移动
  {

    TIM_SetCompare1(TIM4, 23);
    //			-------------------------------------------------------------------------------------------------------------------------------

    EncoderProcess(&Hook_Encoder, Motor_Feedback.Motor_205_Agree);

    HookSpeedPID(&Hook_SpeedPID, 0, Motor_Feedback.Motor_205_Speed);

    Set_Hook_Armour_Speed(CAN2, Hook_SpeedPID.PIDout, 0, 0, 0);

    //			-------------------------------------------------------------------------------------------------------------------------------

    GetXYWSpeed(FORWARD, PanPIDMode);

    MecanumCalculation(Buffer);

    LimitWheelSpeed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

    //速度pid
    PANSpeedPID(&CM1PID, ArmatureRotateSpeed[0],
                Motor_Feedback.Motor_201_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM2PID, ArmatureRotateSpeed[1],
                Motor_Feedback.Motor_202_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM3PID, ArmatureRotateSpeed[2],
                Motor_Feedback.Motor_203_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM4PID, ArmatureRotateSpeed[3],
                Motor_Feedback.Motor_204_Speed * 2 * 3.14 /
                    60); //都是rad/s 反馈转子转速

    Set_CM_Speed(CAN1, CM1PID.PIDout, CM2PID.PIDout, CM3PID.PIDout,
                 CM4PID.PIDout); //得到电流发送给电调
  }
  //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  else if (DBUS_ReceiveData.switch_left == 3) {
    TIM_SetCompare1(TIM4, 5);

    EncoderProcess(&Hook_Encoder, Motor_Feedback.Motor_205_Agree);

    HookFeedAngle = Hook_Encoder.ecd_angle;

    HookSpeedPID(&Hook_SpeedPID, DBUS_ReceiveData.ch2,
                 Motor_Feedback.Motor_205_Speed);

    Set_Hook_Armour_Speed(CAN2, Hook_SpeedPID.PIDout, 0, 0, 0);

    //=======移动=============================================================================

    GetXYWSpeed(BACKWARD, PanPIDMode);

    MecanumCalculation(Buffer);

    LimitWheelSpeed(Buffer, ArmatureRotateSpeed, MAXWHEELSPEED);

    //速度pid
    PANSpeedPID(&CM1PID, ArmatureRotateSpeed[0],
                Motor_Feedback.Motor_201_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM2PID, ArmatureRotateSpeed[1],
                Motor_Feedback.Motor_202_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM3PID, ArmatureRotateSpeed[2],
                Motor_Feedback.Motor_203_Speed * 2 * 3.14 / 60);
    PANSpeedPID(&CM4PID, ArmatureRotateSpeed[3],
                Motor_Feedback.Motor_204_Speed * 2 * 3.14 /
                    60); //都是rad/s 反馈转子转速

    Set_CM_Speed(CAN1, CM1PID.PIDout, CM2PID.PIDout, CM3PID.PIDout,
                 CM4PID.PIDout); //得到电流发送给电调
  }
}

void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    // mainTask();
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }
}
