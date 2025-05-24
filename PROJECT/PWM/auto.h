#ifndef __AUTO_H
#define __AUTO_H

#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <stdint.h>
#include <stdlib.h>  // 添加头文件


void auto_init(char* pwm_value);
void auto_pwm(void);

#endif
