#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "wit_c_sdk.h"


void Usart2Init(unsigned int uiBaud)//USART2(����2)��ʼ������
{
 	GPIO_InitTypeDef GPIO_InitStructure;//GPIO����
	USART_InitTypeDef USART_InitStructure;//USART����
	NVIC_InitTypeDef NVIC_InitStructure; //�ж�NVIC����
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʱ������GPIOA
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//ʱ������USART2
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʱ������GPIOA
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);//ʱ������USART2
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//GPIO��������pin1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_1);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);      
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	 
	USART_InitStructure.USART_BaudRate = uiBaud;//USART����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//�ж�����USART2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART2_IRQHandler(void)
{
	unsigned char ucTemp;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//�ж��Ƿ������ݽ���
	{
		ucTemp = USART_ReceiveData(USART2);//�������ݲ���ֵ
		WitSerialDataIn(ucTemp);//���յ������ݴ���WitSerialDataIn������
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);//�ѽ��յ�����������

	}
}
void Uart2Send(unsigned char *p_data, unsigned int uiSize)//����2���ݷ���
{	
	unsigned int i;
	for(i = 0; i < uiSize; i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);//�ж����ݷ����Ƿ����
		USART_SendData(USART2, *p_data++);//����2���ݷ���		
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}
