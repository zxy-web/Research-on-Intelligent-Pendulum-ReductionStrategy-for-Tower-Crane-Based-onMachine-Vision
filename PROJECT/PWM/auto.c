#include "auto.h"
#include <ctype.h>  
#include "motor.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>  // 添加头文件

int x_pwm=0;
int y_pwm=0;
int z_pwm=0;

void auto_init(char* pwm_value){
  char *xPos;
	char *yPos;
	char *zPos;
	
	 xPos = strstr(pwm_value, "x=");
   yPos = strstr(pwm_value, "y=");
	 zPos = strstr(pwm_value, "z=");
		

    if (xPos != NULL && yPos != NULL)
         {
             y_pwm = atoi(xPos + 2);
             z_pwm = atoi(yPos + 2);
						 x_pwm = atoi(zPos + 2);
                
         }
	
}

void auto_pwm(){
	//左右转		
		if(z_pwm!=0){
			if(z_pwm<0){
				Motor3_rot(1);	
			}
			else{
				Motor3_rot(0);
			}
			if(-5<=z_pwm&&z_pwm<=5){
				Motor3Stop();
			}
			else{
			TIM_SetCompare1(TIM14,abs(z_pwm));
//			TIM_SetCompare1(TIM14,90);
			}
		}
		else{
			Motor3Stop();
		}
		
//挂钩上下		
		if(x_pwm!=0){
			if(x_pwm<0){
				Motor2_rot(0);
			}
			else{
				Motor2_rot(1);
			}
			TIM_SetCompare1(TIM12,abs(x_pwm*4));
//			TIM_SetCompare1(TIM12,0);
		}
		else{
			Motor2Stop();
		}
		
//小车
		if(y_pwm!=0){
			if(y_pwm<0){
				Motor1_rot(1);
			}
			else{
				Motor1_rot(0);
			}
			if(-5<=y_pwm&&y_pwm<=5){
				Motor1Stop();
			}
			else{
			TIM_SetCompare1(TIM13,abs(y_pwm));
//			TIM_SetCompare1(TIM13,50);
			}
		}
		else{
			Motor1Stop();
		}
}
