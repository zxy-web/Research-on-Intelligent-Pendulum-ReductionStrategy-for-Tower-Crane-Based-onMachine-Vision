//把TIM4理解为一个计数器而不是一个定时器，则没有了时序信号。
//这里TIM4的时钟信号（或者说是计数信号）将由电机编码器输出的脉冲代替，也就是说电机脉冲信号成为TIM4的信号，电机每产生一个脉冲被TIM4检测到，则计数器CNT加一（类比于时序信号时每隔一个时间段计数值加一）
//这样的话，输入捕获的自动重装载值period则影响着脉冲值计数到多少之后就溢出，比如65535的话，则接收到65535个脉冲信号之后计数值置零溢出
//这样的话，输入捕获的预分频系数prescaler的作用是，当我不分频时，来一个电机脉冲信号我计数值就加一，当我二分频时，只有接收到两个脉冲信号我才认为是一个有效脉冲，计数值才加一，简单来说就是计数值总体除以二了
//这样我们就把输入捕获初始化完成了，接下来是编码器模式的初始化
//设为TIM_EncoderMode_TI12模式，即计数器在TI1和TI2上升沿处均计数，再根据设置的极性是TIM_ICPolarity_Rising，也就是在TI1和TI2的上升沿计数器累加（或累减）-->那么到时候要除以二
//这样，编码器的初始化就完成了，接下来我们只要通过函数得出它的计数值，就可以知道电机产生的脉冲数，再根据电机的参数（每转产生多少个脉冲）就可以得到电机转了几圈
#include "encode.h"
#include "sys.h"

/**
 * TIM1定时器编码器模式初始化
 * 入口参数：
 * @arr: 自动重装载值，此处为16位自动重装载寄存器
 * @psc: 预分频数值，不分频 
 */
 int Encoder_Timer_Overflow=0;
void Encoder_Init_TIM4(u16 arr,u16 psc)
{
GPIO_InitTypeDef GPIO_InitStructure;
//NVIC_InitTypeDef  NVIC_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;    

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);    
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);  
		
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;          
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                    
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;              
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;                
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                  
  GPIO_Init(GPIOD, &GPIO_InitStructure);                          

  GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);           
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);           
	
  TIM_TimeBaseStructure.TIM_Period = arr; 	                      
	TIM_TimeBaseStructure.TIM_Prescaler=psc;                        
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;       
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;           
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);                  
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;                  
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            
  TIM_ICInitStructure.TIM_ICFilter =0;                            
  TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;                  
  TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;	      
  TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;   
  TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;	            
  TIM_ICInitStructure.TIM_ICFilter=0;                             
  TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising );
		                                 
		
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);                        
	TIM_Cmd(TIM4,ENABLE);   
}


u32 read_cnt2(void)
{
uint32_t encoder_cnt;
	encoder_cnt = TIM4->CNT;		//读取计数器CNT的值，CNT系uint32_t型的变量
	TIM_SetCounter(TIM4, 0);		//每一次读取完计数值后将计数值清零，重新开始累加脉冲，方便下一次计数
	return encoder_cnt;				//返回的值就是本次读到的计数值
}




