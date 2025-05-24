//��TIM4���Ϊһ��������������һ����ʱ������û����ʱ���źš�
//����TIM4��ʱ���źţ�����˵�Ǽ����źţ����ɵ�������������������棬Ҳ����˵��������źų�ΪTIM4���źţ����ÿ����һ�����屻TIM4��⵽���������CNT��һ�������ʱ���ź�ʱÿ��һ��ʱ��μ���ֵ��һ��
//�����Ļ������벶����Զ���װ��ֵperiod��Ӱ��������ֵ����������֮������������65535�Ļ�������յ�65535�������ź�֮�����ֵ�������
//�����Ļ������벶���Ԥ��Ƶϵ��prescaler�������ǣ����Ҳ���Ƶʱ����һ����������ź��Ҽ���ֵ�ͼ�һ�����Ҷ���Ƶʱ��ֻ�н��յ����������ź��Ҳ���Ϊ��һ����Ч���壬����ֵ�ż�һ������˵���Ǽ���ֵ������Զ���
//�������ǾͰ����벶���ʼ������ˣ��������Ǳ�����ģʽ�ĳ�ʼ��
//��ΪTIM_EncoderMode_TI12ģʽ������������TI1��TI2�����ش����������ٸ������õļ�����TIM_ICPolarity_Rising��Ҳ������TI1��TI2�������ؼ������ۼӣ����ۼ���-->��ô��ʱ��Ҫ���Զ�
//�������������ĳ�ʼ��������ˣ�����������ֻҪͨ�������ó����ļ���ֵ���Ϳ���֪��������������������ٸ��ݵ���Ĳ�����ÿת�������ٸ����壩�Ϳ��Եõ����ת�˼�Ȧ
#include "encode.h"
#include "sys.h"

/**
 * TIM1��ʱ��������ģʽ��ʼ��
 * ��ڲ�����
 * @arr: �Զ���װ��ֵ���˴�Ϊ16λ�Զ���װ�ؼĴ���
 * @psc: Ԥ��Ƶ��ֵ������Ƶ 
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
	encoder_cnt = TIM4->CNT;		//��ȡ������CNT��ֵ��CNTϵuint32_t�͵ı���
	TIM_SetCounter(TIM4, 0);		//ÿһ�ζ�ȡ�����ֵ�󽫼���ֵ���㣬���¿�ʼ�ۼ����壬������һ�μ���
	return encoder_cnt;				//���ص�ֵ���Ǳ��ζ����ļ���ֵ
}




