//把TIM3理解为一个计数器而不是一个定时器，则没有了时序信号。
//这里TIM3的时钟信号（或者说是计数信号）将由电机编码器输出的脉冲代替，也就是说电机脉冲信号成为TIM3的信号，电机每产生一个脉冲被TIM3检测到，则计数器CNT加一（类比于时序信号时每隔一个时间段计数值加一）
//这样的话，输入捕获的自动重装载值period则影响着脉冲值计数到多少之后就溢出，比如65535的话，则接收到65535个脉冲信号之后计数值置零溢出
//这样的话，输入捕获的预分频系数prescaler的作用是，当我不分频时，来一个电机脉冲信号我计数值就加一，当我二分频时，只有接收到两个脉冲信号我才认为是一个有效脉冲，计数值才加一，简单来说就是计数值总体除以二了
//这样我们就把输入捕获初始化完成了，接下来是编码器模式的初始化
//设为TIM_EncoderMode_TI12模式，即计数器在TI1和TI2上升沿处均计数，再根据设置的极性是TIM_ICPolarity_Rising，也就是在TI1和TI2的上升沿计数器累加（或累减）-->那么到时候要除以二
//这样，编码器的初始化就完成了，接下来我们只要通过函数得出它的计数值，就可以知道电机产生的脉冲数，再根据电机的参数（每转产生多少个脉冲）就可以得到电机转了几圈
#include "encode.h"
#include "sys.h"

//初始化编码器
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
	//也就是说，这里的TIM3的时钟信号其实是由A/B相的频率来决定的，类似于外部时钟，然后分频就是对这个脉冲频率分频，比如二分频就是把两个脉冲记为一个脉冲。
	TIM_TimeBaseStructure.TIM_Prescaler = 1;					//这里我们把它设为1，即不分频
	TIM_TimeBaseStructure.TIM_Period = 65535;					//每来一个脉冲信号的上升沿（下面有设置）计数值就累加（或累减），65535则为最大计数值，就溢出了
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //这里按理来说应该不起作用，因为计数方向是受TI1和TI2信号的影响的
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_ICStructInit(&TIM_ICInitStructure);						//Fills each TIM_ICInitStruct member with its default value
	//相当于：
	//	void TIM_ICStructInit(TIM_ICInitTypeDef* TIM_ICInitStruct)
	//	{
	//	  /* Set the default configuration */
	//	  TIM_ICInitStruct->TIM_Channel = TIM_Channel_1;
	//	  TIM_ICInitStruct->TIM_ICPolarity = TIM_ICPolarity_Rising;
	//	  TIM_ICInitStruct->TIM_ICSelection = TIM_ICSelection_DirectTI;
	//	  TIM_ICInitStruct->TIM_ICPrescaler = TIM_ICPSC_DIV1;
	//	  TIM_ICInitStruct->TIM_ICFilter = 0x00;
	//	}
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising ,TIM_ICPolarity_Rising); //配置为编码器模式，计数器在TI1和TI2上升沿处均计数

	TIM_SetCounter(TIM3, 0);		//将脉冲计数值设为零
	TIM_Cmd(TIM3, ENABLE);			//使能TIM3
}

// 读取定时器计数值
uint32_t read_cnt(void)
{
	uint32_t encoder_cnt;
	encoder_cnt = TIM3->CNT;		//读取计数器CNT的值，CNT系uint32_t型的变量
	TIM_SetCounter(TIM3, 0);		//每一次读取完计数值后将计数值清零，重新开始累加脉冲，方便下一次计数
	return encoder_cnt;				//返回的值就是本次读到的计数值
}


