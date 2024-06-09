#include "stm32f4xx.h"
#include "Serial.h"


void Serial_SendByte(uint8_t Byte)
{	
		USART_SendData(UART7, Byte);
		
	  while (USART_GetFlagStatus(UART7, USART_FLAG_TXE) == RESET);
		
}

//void Serial_SendArray(uint8_t *Array, uint16_t Length)
//{
//		uint16_t i;
//		for	(i = 0; i < Length; i ++)
//		{
//		
//		
//		
//		}

//}