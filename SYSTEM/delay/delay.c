#include "delay.h"
#include "misc.h"

static uint16_t  fac_us=0;
static uint16_t fac_ms=0;

static volatile uint32_t sv_uiDelay = 0;
void SysTick_Init(void)//ʱ�ӳ�ʼ�������Ҫ���ݲ�ͬ�ĵ�Ƭ��������������
{	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//ѡ��8��Ƶ
	fac_us=SystemCoreClock/8000000;//SystemCoreClockѡ��72MHz��72000000���ý����9MHz,ʱ�䵥λ����1/(9MHz)=1/(9us)
	fac_ms=(uint16_t)fac_us*1000;//1us*1000=1ms
}

void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;
	SysTick->VAL =0x00;
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
	SysTick->VAL =0X00;	    
} 
