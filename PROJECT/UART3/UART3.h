#ifndef __UART3_H
#define __UART3_H
#include "stm32f4xx_usart.h"
#include "stdint.h"
#include "sys.h"
typedef unsigned char u8;
typedef unsigned short u16;

#define USART3_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART3_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define BUFFER3_SIZE 128

extern u8  USART3_RX_BUF[USART3_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART3_RX_STA;         		//����״̬���	

void Usart3Init(unsigned int uiBaud);

#endif

