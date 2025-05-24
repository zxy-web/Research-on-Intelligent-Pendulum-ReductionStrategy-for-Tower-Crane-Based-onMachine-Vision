#include "sys.h"
#include "usart_3.h"
#include "usart.h"


static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
static unsigned char count=0; 
extern void CopeSerial3Data(unsigned char ucData);

void uart3_init(unsigned long baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
    
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA10复用为USART1 USART3_TX
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA9复用为USART1 USART3_RX
	
	//USART1端口配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PB10，PB11
	 
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    
	USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);    
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
	USART_ClearFlag(USART3,USART_FLAG_TC);	
	USART_Cmd(USART3, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void USART3_IRQHandler(void)
{
  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET) //USART_IT_TXE为1时发送寄存器空
  {   
    USART_SendData(USART3, TxBuffer[TxCounter++]); 
    USART_ClearITPendingBit(USART3, USART_IT_TXE);
    if(TxCounter == count) USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  }
	else if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)//USART_IT_RXNE为1时接受数据寄存器有数据
	
  {
		CopeSerial3Data((unsigned char)USART3->DR);//处理数据
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
	
	USART_ClearITPendingBit(USART3,USART_IT_ORE);
}


void UART3_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;  
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);  
}

void UART3_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str=='\r')UART3_Put_Char(0x0d);
			else if(*Str=='\n')UART3_Put_Char(0x0a);
				else UART3_Put_Char(*Str);
		Str++;
	}
}

