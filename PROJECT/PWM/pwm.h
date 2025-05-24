#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"


void TIM14_PWM_Init(u32 arr,u32 psc); //旋转PF9

void TIM13_PWM_Init(u32 arr,u32 psc); //小车PF8

void TIM12_PWM_Init(u32 arr,u32 psc); //PB14上下


#endif
