
#include "main.h"

#include "bsp_usart.h"
#include "stdio.h"
#include "string.h"
#include "stdarg.h"
#include <protocol.h>


#define MAX_LEN 100
uint8_t g_dma_buff[MAX_LEN] = {0};

extern UART_HandleTypeDef huart1;


void USART1_Send_U8(uint8_t ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
}


void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	if (!Length) return;
	while (Length--)
	{
		USART1_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}


void USART1_Send_ArrayU8_DMA(uint8_t *BufferPtr, uint16_t Length)
{
	if (!Length) return;

	if (BufferPtr)
	{
		memcpy(g_dma_buff, BufferPtr, (Length > MAX_LEN ? MAX_LEN : Length));
		HAL_UART_Transmit_DMA(&huart1, g_dma_buff, Length);
	}
}


