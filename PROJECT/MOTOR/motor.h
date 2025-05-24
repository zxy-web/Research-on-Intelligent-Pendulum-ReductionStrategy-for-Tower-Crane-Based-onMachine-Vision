#ifndef __MOTOR_H

#define __MOTOR_H

#include "sys.h"
#define motor1IN1 PBout(6) // hua che
#define motor1IN2 PBout(7) 
#define motor2IN3 PDout(6) //shang xia
#define motor2IN4 PDout(7) 
#define motor3IN5 PDout(2) // yaw axis
#define motor3IN6 PDout(3)
#define EN_A PBout(3) 
#define EN_B PBout(5) 
#define EN_C PBout(4) 



void MOTOR_Init(void);//MOTOR

void MotorForward(void);
void MotorReverse(void);
void Motor1Stop(void);

void Motor2Stop(void);
void Motor3Stop(void);


void Motor1_rot(unsigned int dir);
void Motor2_rot(unsigned int dir);
void Motor3_rot(unsigned int dir);



#endif 
