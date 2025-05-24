
#include "motor.h"
#include "stm32f4xx.h"
#include "sys.h"

void MotorForward(void)//??
{
    motor1IN1=1;
		motor1IN2=0;
		motor2IN3=1;
		motor2IN4=0;
	  motor3IN5=1;
		motor3IN6=0;
}
void MotorReverse(void)//??
{
    motor1IN1=0;
		motor1IN2=1;
		motor2IN3=0;
		motor2IN4=1;
	  motor3IN5=0;
		motor3IN6=1;
}
void Motor1Stop(void)//??
{
    motor1IN1=0;
		motor1IN2=0;
		
}

void Motor2Stop(void)//??
{
		motor2IN3=0;
		motor2IN4=0;
}

void Motor3Stop(void)//??
{
		motor3IN5=0;
		motor3IN6=0;
}

void Motor1_rot(unsigned int dir)
{
	if(dir == 1)
	{	
		motor1IN1 = 0;
		motor1IN2 = 1;
	}
	else
	{
		motor1IN1 = 1;
		motor1IN2 = 0;
	}
}

void Motor2_rot(unsigned int dir)
{
	if(dir == 1)
	{	
		motor2IN3 = 0;
		motor2IN4 = 1;
	}
	else
	{
		motor2IN3 = 1;
		motor2IN4 = 0;
	}
}

void Motor3_rot(unsigned int dir)
{
	if(dir == 1)
	{	
		motor3IN5 = 0;
		motor3IN6 = 1;
	}
	else
	{
		motor3IN5 = 1;
		motor3IN6 = 0;
	}
}



void MOTOR_Init(void)//???
{
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_6|GPIO_Pin_7);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	GPIO_ResetBits(GPIOD,GPIO_Pin_2|GPIO_Pin_3);
}
