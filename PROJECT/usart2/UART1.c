#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "UART1.h"
#include <string.h>
#include <stdlib.h>  // 添加头文件
#include <motor.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>
#include "delay.h"

#define RX_BUFFER_SIZE 32

char uart1_rx_buf[RX_BUFFER_SIZE];
uint8_t uart1_rx_index = 0;
uint8_t uart1_rx_ready = 0;

// 控制状态变量
int xss = 0; // 0停止，1上升，2下降
int yht = 0; // 0停止，1后退，2前进
int zyz = 0; // 0停止，1右转，2左转
int sd  = 2; // 2未初始化，0手动，1自动
int dir = 0; // 自动时目标：0地板，1箱子

// ===================== 串口初始化 =====================
void Usart1Init(unsigned int uiBaud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  // TX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // RX

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = uiBaud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

// ===================== 串口中断 =====================
void USART1_IRQHandler(void)
{
    uint8_t ch;

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        ch = USART_ReceiveData(USART1);
			
				if (ch >= 0x61 && ch <= 0x67) {
				uart1_rx_buf[0] = ch;
				uart1_rx_buf[1] = '\0';
				uart1_rx_ready = 1;
					uart1_rx_index = 0;
				}

        if (ch == '\n')
        {
            uart1_rx_buf[uart1_rx_index] = '\0';
            uart1_rx_index = 0;
            uart1_rx_ready = 1;
        }
        else
        {
            if (uart1_rx_index < RX_BUFFER_SIZE - 1)
            {
                uart1_rx_buf[uart1_rx_index++] = ch;
            }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

// ===================== 串口接收指令解析 =====================
void process_uart1_command(void)
{
    int pwm_x = 0, pwm_phi = 0;

    if (!uart1_rx_ready)
        return;

    uart1_rx_ready = 0;

    if (strchr(uart1_rx_buf, ',') != NULL)
    {
        // 自动控制：格式 "65,30"
        if (sscanf(uart1_rx_buf, "%d,%d", &pwm_x, &pwm_phi) == 2)
        {
            sd = 1; // 切换自动模式
						printf("%d,%d",pwm_x,pwm_phi);
            if (pwm_x < 0){
							pwm_x = -pwm_x;
							TIM_SetCompare1(TIM13, pwm_x);
							Motor1_rot(1); //后退
							
						}
						else {
							TIM_SetCompare1(TIM13, pwm_x);
							Motor1_rot(0); //前进
						
						}
						
						
            if (pwm_phi < 0){
							pwm_phi = -pwm_phi;
							TIM_SetCompare1(TIM14, pwm_phi);
							Motor3_rot(1);  //往右旋转
						} 
            
						else{
							TIM_SetCompare1(TIM14, pwm_phi);	
							Motor3_rot(0);  //往左旋转							
						}
						
       }
 }
		else
			{
        // 手动控制：单字节命令
        uint8_t cmd = uart1_rx_buf[0];
        sd = 0;
        switch (cmd)
        {
            case 0x61: yht = 2; break;
            case 0x62: yht = 1; break;
            case 0x63: zyz = 2; break;
            case 0x64: zyz = 1; break;
            case 0x65: xss = 1; break;
            case 0x66: xss = 2; break; 
            case 0x67: xss = 0; yht = 0; zyz = 0; sd = 0; break; // 停止，回手动
            default: break;
        }
				
			if(xss==1){
						Motor2_rot(0);
						TIM_SetCompare1(TIM12,99);
						delay_ms(1000);
		}
		else if(xss==2){
						Motor2_rot(1);
						TIM_SetCompare1(TIM12,99);
						delay_ms(1000);
		}
		else{
					Motor2Stop();
					TIM_SetCompare1(TIM12,0);
					delay_ms(1000);
		}
		if(yht==1){
					Motor1_rot(0);
					TIM_SetCompare1(TIM13,99);
					delay_ms(1000);
		}
		else if(yht==2){
					Motor1_rot(1);
					TIM_SetCompare1(TIM13,99);
					delay_ms(1000);
		}
		else{
					TIM_SetCompare1(TIM13,0);
					Motor1Stop();
					delay_ms(1000);
		}
		if(zyz==1){
					Motor3_rot(1);
					TIM_SetCompare1(TIM14,99);//修改了
					delay_ms(1000);
		}
		else if(zyz==2){
				Motor3_rot(0);
				TIM_SetCompare1(TIM14,99);
				delay_ms(1000);
		}
		else{
			TIM_SetCompare1(TIM14,0);
				Motor3Stop();
			delay_ms(1000);
			
		}
    }
}


int fputc(int ch, FILE *file)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
	USART_SendData(USART1, ch);
	return ch;
}

void CopeCmdData(unsigned char ucData);


#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
