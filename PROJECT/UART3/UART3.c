#include "UART3.h"
#include "UART1.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <string.h>
#include <stdlib.h>  // ���ͷ�ļ�
#include "auto.h"

uint8_t rxBuffer3[BUFFER3_SIZE]; //���յ�������
uint16_t rxBuffer3Index = 0;    //���յ��ĳ���
uint8_t usart3RxComplete = 0; //�Ƿ���յ�����
u16 USART3_RX_STA=0;
void Usart3Init(unsigned int uiBaud){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB10����ΪUSART3	
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB11��GPIOB10��ʼ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��GPIOB11����GPIOB10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = uiBaud;//������ 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
	USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж� 

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	

}

void USART3_IRQHandler(void){
	char rxBuffer3_3[BUFFER3_SIZE];

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
    rxBuffer3[rxBuffer3Index] =USART_ReceiveData(USART3);//(USART1->DR);	//��ȡ���յ�������
		if ((rxBuffer3[rxBuffer3Index] == '\n') || (rxBuffer3Index >= BUFFER3_SIZE - 1))
        {
            rxBuffer3[rxBuffer3Index] = '\0';  // ��� null ������
            usart3RxComplete = 1;
            rxBuffer3Index = 0;  // ���ý�������

					strncpy(rxBuffer3_3, (char*)rxBuffer3, BUFFER_SIZE);
					
					auto_init(rxBuffer3_3);
					auto_pwm();
					
					memset(rxBuffer3, 0, sizeof(rxBuffer3));
	      	usart3RxComplete=0;

  } 
		else
      {
          rxBuffer3Index++;
     }
}
	}
