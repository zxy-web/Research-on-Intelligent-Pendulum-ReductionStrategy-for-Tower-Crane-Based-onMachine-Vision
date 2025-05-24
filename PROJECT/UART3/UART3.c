#include "UART3.h"
#include "UART1.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <string.h>
#include <stdlib.h>  // 添加头文件
#include "auto.h"

uint8_t rxBuffer3[BUFFER3_SIZE]; //接收到的数据
uint16_t rxBuffer3Index = 0;    //接收到的长度
uint8_t usart3RxComplete = 0; //是否接收到数据
u16 USART3_RX_STA=0;
void Usart3Init(unsigned int uiBaud){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB11复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB10复用为USART3	
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB11和GPIOB10初始化
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化GPIOB11，和GPIOB10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = uiBaud;//波特率 
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
	USART_Cmd(USART3, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
		
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断 

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	

}

void USART3_IRQHandler(void){
	char rxBuffer3_3[BUFFER3_SIZE];

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
    rxBuffer3[rxBuffer3Index] =USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
		if ((rxBuffer3[rxBuffer3Index] == '\n') || (rxBuffer3Index >= BUFFER3_SIZE - 1))
        {
            rxBuffer3[rxBuffer3Index] = '\0';  // 添加 null 结束符
            usart3RxComplete = 1;
            rxBuffer3Index = 0;  // 重置接收索引

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
