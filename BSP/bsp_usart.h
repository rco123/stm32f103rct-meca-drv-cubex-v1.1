#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "main.h"


#define USART1_BAUDRATE     115200

void USART1_Init(uint32_t baudrate);
void USART1_Send_U8(uint8_t ch);
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length);

#endif
