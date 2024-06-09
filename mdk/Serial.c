#include "stm32f4xx.h"

void Serial_Init(void)
{

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);	      
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;	              
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOG, &GPIO_InitStructure);          	   
  
   	  
	USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;						
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;		
  USART_InitStructure.USART_StopBits = USART_StopBits_1;			
  USART_InitStructure.USART_Parity = USART_Parity_No;				
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;				
  USART_Init(USART2, &USART_InitStructure); 
  
  USART_Cmd(USART6, ENABLE);
  
}

void Serial_SendByte(uint8_t Byte)
{
		
		USART_SendData(USART6, Byte);
		while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
			
}

