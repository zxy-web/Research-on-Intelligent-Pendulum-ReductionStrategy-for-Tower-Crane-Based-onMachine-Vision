#include "stm32f4xx.h"
#include "UART1.h"
#include "UART2.h"
#include "usart_3.h"
#include "delay.h"
#include "pwm.h"
#include "motor.h"
#include "stdlib.h"
#include "stdio.h"
#include <string.h>
#include "math.h"
#include "misc.h"
#include "WT901cttl.h"
#include "encode.h"

extern int xss;//停止的时候为0，上升为1，下降为2
extern int yht;//停止的时候为0，后退为1，前进为2
extern int zyz;//停止的时候为0，右转为1，左转为2
extern int sd;
extern int dir; ///1目标位置为箱子，0是地板
int main(void)
{
	int i=0;
	int flag=0;
	int j=0;
	int array_pwm_car[30] = {70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99};
	int array_pwm_axis[30] ={70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99}; 
	uint32_t cnt_temp;			//用于暂存TIM的计数值，即TIM检测到的脉冲的数量
	float pulse;				//电机产生的实际脉冲值
	float round;				//电机转的圈数
	float x=0;
	
	SysTick_Init();//系统时钟初始化
	Usart1Init(9600);//串口1初始化接PC端
	Usart2Init(9600);//串口2初始化接传感器端
	uart3_init(9600); //陀螺仪数据
	delay_ms(86);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	TIM14_PWM_Init(100-1,84-1); //初始arr=100，分频84
	TIM13_PWM_Init(100-1,84-1);
	TIM12_PWM_Init(100-1,84-1);
	MOTOR_Init();
	TIM_SetCompare1(TIM13,0); //小车
	TIM_SetCompare1(TIM14,0); //旋转
	TIM_SetCompare1(TIM12,0); //挂钩上下
		
	encoder_init(); //TIM3编码器模式初始化，A6、A7分别作为A相和B相的脉冲输入
		
	Encoder_Init_TIM4(65535,1); //TIM4编码器模式初始化，D12、D13分别作为A相和B相的脉冲输入

    while(1)
	{
		if(sd==1){
			Motor2Stop();
		}
		
		DATA_OUT_PUT(j);
		
		//后退是-，前进是+
		
		cnt_temp = read_cnt();         //得到脉冲计数值
		if(cnt_temp>10000){
		printf("forward\r\n");
		cnt_temp=65535-cnt_temp;
		pulse = cnt_temp/4.0f;						//由于是TIM_EncoderMode_TI12，所以要四分频，即除以四，得到实际的脉冲值
		round = cnt_temp/4.0f/1000.0f;				//假设电机每转产生260个脉冲，则通过该公式可求出电机转了几圈
		//x=x+round *200.0f;
		pulse=-pulse;
		
		printf("cnt_temp:%d\r\n", cnt_temp);		//向串口打印脉冲计数值
		printf("pulse:%f\r\n", pulse);				//向串口打印实际脉冲值
		printf("round:%f\r\n", round);				//向串口打印电机转了几圈
		if(pulse<-10){
			x = x+pulse;
			printf("x:%f\r\n", x);  //向串口打印累计位移
		}
		else{
			x = x;
			printf("x:%f\r\n", x);
		}
		                
		}
		else	{
		printf("backward\r\n");
		
		pulse = cnt_temp/4.0f;						//由于是TIM_EncoderMode_TI12，所以要四分频，即除以四，得到实际的脉冲值
		round = cnt_temp/4.0f/1000.0f;				//假设电机每转产生260个脉冲，则通过该公式可求出电机转了几圈
		//x=x-round *200.0f;
		printf("cnt_temp:%d\r\n", cnt_temp);		//向串口打印脉冲计数值
		printf("pulse:%f\r\n", pulse);				//向串口打印实际脉冲值
		if(pulse>10){
			x = x+pulse;
			printf("x:%f\r\n", x);  //向串口打印累计位移
		}
		else{
			x=x;
			printf("x:%f\r\n", x);
		}
		}
		
		process_uart1_command();
}
}


