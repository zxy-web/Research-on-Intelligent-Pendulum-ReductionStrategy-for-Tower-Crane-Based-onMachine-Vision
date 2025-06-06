#ifndef __UART3_H
#define __UART3_H
#include "stm32f4xx_usart.h"
#include "stdint.h"
#include "sys.h"
typedef unsigned char u8;
typedef unsigned short u16;

#define USART3_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART3_RX 			1		//使能（1）/禁止（0）串口1接收
#define BUFFER3_SIZE 128

extern u8  USART3_RX_BUF[USART3_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART3_RX_STA;         		//接收状态标记	

void Usart3Init(unsigned int uiBaud);

#endif

