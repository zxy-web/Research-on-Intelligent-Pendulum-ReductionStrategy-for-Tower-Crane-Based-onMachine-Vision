#ifndef __ENCODE2_H
#define __ENCODE2_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"

void Encoder_Init_TIM4(u16 arr,u16 psc);
u32 read_cnt2(void);
#endif
