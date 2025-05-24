#ifndef __UART1_H
#define __UART1_H

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define BUFFER_SIZE 128
#define RX_BUFFER_SIZE 32
typedef unsigned char u8;
typedef unsigned short u16;
#include <stdint.h>

	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	

// ===================== 变量声明 =====================
extern char uart1_rx_buf[RX_BUFFER_SIZE];
extern uint8_t uart1_rx_index;
extern uint8_t uart1_rx_ready;

void Usart1Init(unsigned int uiBaud);
void process_uart1_command(void);

#endif

//------------------End of File----------------------------

