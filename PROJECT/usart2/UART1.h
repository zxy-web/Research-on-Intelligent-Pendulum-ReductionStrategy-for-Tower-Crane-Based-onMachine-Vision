#ifndef __UART1_H
#define __UART1_H

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define BUFFER_SIZE 128
#define RX_BUFFER_SIZE 32
typedef unsigned char u8;
typedef unsigned short u16;
#include <stdint.h>

	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	

// ===================== �������� =====================
extern char uart1_rx_buf[RX_BUFFER_SIZE];
extern uint8_t uart1_rx_index;
extern uint8_t uart1_rx_ready;

void Usart1Init(unsigned int uiBaud);
void process_uart1_command(void);

#endif

//------------------End of File----------------------------

