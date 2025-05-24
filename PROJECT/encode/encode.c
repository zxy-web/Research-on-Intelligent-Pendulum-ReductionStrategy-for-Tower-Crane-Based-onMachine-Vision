//��TIM3���Ϊһ��������������һ����ʱ������û����ʱ���źš�
//����TIM3��ʱ���źţ�����˵�Ǽ����źţ����ɵ�������������������棬Ҳ����˵��������źų�ΪTIM3���źţ����ÿ����һ�����屻TIM3��⵽���������CNT��һ�������ʱ���ź�ʱÿ��һ��ʱ��μ���ֵ��һ��
//�����Ļ������벶����Զ���װ��ֵperiod��Ӱ��������ֵ����������֮������������65535�Ļ�������յ�65535�������ź�֮�����ֵ�������
//�����Ļ������벶���Ԥ��Ƶϵ��prescaler�������ǣ����Ҳ���Ƶʱ����һ����������ź��Ҽ���ֵ�ͼ�һ�����Ҷ���Ƶʱ��ֻ�н��յ����������ź��Ҳ���Ϊ��һ����Ч���壬����ֵ�ż�һ������˵���Ǽ���ֵ������Զ���
//�������ǾͰ����벶���ʼ������ˣ��������Ǳ�����ģʽ�ĳ�ʼ��
//��ΪTIM_EncoderMode_TI12ģʽ������������TI1��TI2�����ش����������ٸ������õļ�����TIM_ICPolarity_Rising��Ҳ������TI1��TI2�������ؼ������ۼӣ����ۼ���-->��ô��ʱ��Ҫ���Զ�
//�������������ĳ�ʼ��������ˣ�����������ֻҪͨ�������ó����ļ���ֵ���Ϳ���֪��������������������ٸ��ݵ���Ĳ�����ÿת�������ٸ����壩�Ϳ��Եõ����ת�˼�Ȧ
#include "encode.h"
#include "sys.h"

//��ʼ��������
void encoder_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);    
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	
	//Specifies the prescaler value used to divide the TIM clock.
	//Ҳ����˵�������TIM3��ʱ���ź���ʵ����A/B���Ƶ���������ģ��������ⲿʱ�ӣ�Ȼ���Ƶ���Ƕ��������Ƶ�ʷ�Ƶ���������Ƶ���ǰ����������Ϊһ�����塣
	TIM_TimeBaseStructure.TIM_Prescaler = 1;					//�������ǰ�����Ϊ1��������Ƶ
	TIM_TimeBaseStructure.TIM_Period = 65535;					//ÿ��һ�������źŵ������أ����������ã�����ֵ���ۼӣ����ۼ�����65535��Ϊ������ֵ���������
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ﰴ����˵Ӧ�ò������ã���Ϊ������������TI1��TI2�źŵ�Ӱ���
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ICStructInit(&TIM_ICInitStructure);						//Fills each TIM_ICInitStruct member with its default value
	//�൱�ڣ�
	//	void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct)
	//	{
	//	  /* Set the default configuration */
	//	  TIM_ICInitStruct->TIM_Channel = TIM_Channel_1;
	//	  TIM_ICInitStruct->TIM_ICPolarity = TIM_ICPolarity_Rising;
	//	  TIM_ICInitStruct->TIM_ICSelection = TIM_ICSelection_DirectTI;
	//	  TIM_ICInitStruct->TIM_ICPrescaler = TIM_ICPSC_DIV1;
	//	  TIM_ICInitStruct->TIM_ICFilter = 0x00;
	//	}
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); //����Ϊ������ģʽ����������TI1��TI2�����ش�������

	TIM_SetCounter(TIM3, 0);		//���������ֵ��Ϊ��
	TIM_Cmd(TIM3, ENABLE);			//ʹ��TIM3
}

// ��ȡ��ʱ������ֵ
uint32_t read_cnt(void)
{
	uint32_t encoder_cnt;
	encoder_cnt = TIM3->CNT;		//��ȡ������CNT��ֵ��CNTϵuint32_t�͵ı���
	TIM_SetCounter(TIM3, 0);		//ÿһ�ζ�ȡ�����ֵ�󽫼���ֵ���㣬���¿�ʼ�ۼ����壬������һ�μ���
	return encoder_cnt;				//���ص�ֵ���Ǳ��ζ����ļ���ֵ
}


