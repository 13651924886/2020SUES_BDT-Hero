#ifndef __UART8_H__
#define __UART8_H__ 
#include "main.h"
#include <stm32f4xx.h> 
#include <stdio.h>


void LX_IMU_UART8_Configuration(u32 bound);
void Uart8_Send( unsigned char *DataToSend , u8 data_num );

void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch);

#endif
