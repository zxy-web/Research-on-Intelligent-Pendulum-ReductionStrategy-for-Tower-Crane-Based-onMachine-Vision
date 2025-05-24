#ifndef __ENCODE_H
#define __ENCODE_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
void encoder_init(void);
uint32_t read_cnt(void);

#endif
