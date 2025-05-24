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

extern int xss;//ֹͣ��ʱ��Ϊ0������Ϊ1���½�Ϊ2
extern int yht;//ֹͣ��ʱ��Ϊ0������Ϊ1��ǰ��Ϊ2
extern int zyz;//ֹͣ��ʱ��Ϊ0����תΪ1����תΪ2
extern int sd;
extern int dir; ///1Ŀ��λ��Ϊ���ӣ�0�ǵذ�
int main(void)
{
	int i=0;
	int flag=0;
	int j=0;
	int array_pwm_car[30] = {70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99};
	int array_pwm_axis[30] ={70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99}; 
	uint32_t cnt_temp;			//�����ݴ�TIM�ļ���ֵ����TIM��⵽�����������
	float pulse;				//���������ʵ������ֵ
	float round;				//���ת��Ȧ��
	float x=0;
	
	SysTick_Init();//ϵͳʱ�ӳ�ʼ��
	Usart1Init(9600);//����1��ʼ����PC��
	Usart2Init(9600);//����2��ʼ���Ӵ�������
	uart3_init(9600); //����������
	delay_ms(86);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	TIM14_PWM_Init(100-1,84-1); //��ʼarr=100����Ƶ84
	TIM13_PWM_Init(100-1,84-1);
	TIM12_PWM_Init(100-1,84-1);
	MOTOR_Init();
	TIM_SetCompare1(TIM13,0); //С��
	TIM_SetCompare1(TIM14,0); //��ת
	TIM_SetCompare1(TIM12,0); //�ҹ�����
		
	encoder_init(); //TIM3������ģʽ��ʼ����A6��A7�ֱ���ΪA���B�����������
		
	Encoder_Init_TIM4(65535,1); //TIM4������ģʽ��ʼ����D12��D13�ֱ���ΪA���B�����������

    while(1)
	{
		if(sd==1){
			Motor2Stop();
		}
		
		DATA_OUT_PUT(j);
		
		//������-��ǰ����+
		
		cnt_temp = read_cnt();         //�õ��������ֵ
		if(cnt_temp>10000){
		printf("forward\r\n");
		cnt_temp=65535-cnt_temp;
		pulse = cnt_temp/4.0f;						//������TIM_EncoderMode_TI12������Ҫ�ķ�Ƶ���������ģ��õ�ʵ�ʵ�����ֵ
		round = cnt_temp/4.0f/1000.0f;				//������ÿת����260�����壬��ͨ���ù�ʽ��������ת�˼�Ȧ
		//x=x+round *200.0f;
		pulse=-pulse;
		
		printf("cnt_temp:%d\r\n", cnt_temp);		//�򴮿ڴ�ӡ�������ֵ
		printf("pulse:%f\r\n", pulse);				//�򴮿ڴ�ӡʵ������ֵ
		printf("round:%f\r\n", round);				//�򴮿ڴ�ӡ���ת�˼�Ȧ
		if(pulse<-10){
			x = x+pulse;
			printf("x:%f\r\n", x);  //�򴮿ڴ�ӡ�ۼ�λ��
		}
		else{
			x = x;
			printf("x:%f\r\n", x);
		}
		                
		}
		else	{
		printf("backward\r\n");
		
		pulse = cnt_temp/4.0f;						//������TIM_EncoderMode_TI12������Ҫ�ķ�Ƶ���������ģ��õ�ʵ�ʵ�����ֵ
		round = cnt_temp/4.0f/1000.0f;				//������ÿת����260�����壬��ͨ���ù�ʽ��������ת�˼�Ȧ
		//x=x-round *200.0f;
		printf("cnt_temp:%d\r\n", cnt_temp);		//�򴮿ڴ�ӡ�������ֵ
		printf("pulse:%f\r\n", pulse);				//�򴮿ڴ�ӡʵ������ֵ
		if(pulse>10){
			x = x+pulse;
			printf("x:%f\r\n", x);  //�򴮿ڴ�ӡ�ۼ�λ��
		}
		else{
			x=x;
			printf("x:%f\r\n", x);
		}
		}
		
		process_uart1_command();
}
}


