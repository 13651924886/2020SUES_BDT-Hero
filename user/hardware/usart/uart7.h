#ifndef __UART7_H__
#define __UART7_H__ 
#include "main.h"
#include <stm32f4xx.h> 


void ANO_DT_UART7_Configuration(u32 bound);
void Uart7_IRQ ( void );
void UART7_Put_Buf( unsigned char *DataToSend, u8 data_num );



#endif

