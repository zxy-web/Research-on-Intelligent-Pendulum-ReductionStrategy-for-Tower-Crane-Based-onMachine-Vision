#ifndef __USART_3_H
#define __USART_3_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"


void uart3_init(unsigned long baudrate);
void UART3_Put_Char(unsigned char DataToSend);
void UART3_Put_String(unsigned char *Str);


#endif

