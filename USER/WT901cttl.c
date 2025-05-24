#include "WT901cttl.h"
#include "usart_3.h"
#include <string.h>
#include <stdio.h>
#include "delay.h"
#include "usart.h"
#include "PID control.h"
#include "motor.h"

struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	stcAngle;
struct SMag 		stcMag;
struct SDStatus stcDStatus;
struct SPress 	stcPress;
struct SLonLat 	stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ       stcQ;

char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//������ٶ�У׼ģʽ
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//���浱ǰ����

float Angle_z_0=0;

//�ô���2��JYģ�鷢��ָ��
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<5;i++)
		UART3_Put_Char(cmd[i]);
}

void CopeSerial3Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;	
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;
	}
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//��ջ�����
	}
}

void CopeSerial1Data(unsigned char ucData)
{	
	UART3_Put_Char(ucData);//ת������1�յ������ݸ�����2��JYģ�飩
}

void DATA_OUT_PUT(int i, int setpoint)
{
	  
	  float Angle_z;
	  float Gyro_z;
		float error1;
		PIDDouble piddouble;	
	  piddouble.out.kp=3;
	  piddouble.out.ki=0.04;
	  piddouble.out.kd=0;
	  piddouble.in.kd=0.3;
	  piddouble.in.kp=9;
	  piddouble.in.ki=0.1;
//	  static int led0pwmval=0;
     delay_ms(1000);
		i++;
		if(i>20)
		{
			i = 0;
			printf("���ڽ��м��ٶ�У׼\r\n");
			sendcmd(ACCCALSW);delay_ms(100);//�ȴ�ģ���ڲ��Զ�У׼�ã�ģ���ڲ����Զ�������Ҫһ����ʱ��
			sendcmd(SAVACALSW);delay_ms(100);//���浱ǰ����
			printf("���ٶ�У׼���\r\n");
		}
		//���ʱ��
//		printf("Time:20%d-%d-%d %d:%d:%.3f\r\n",stcTime.ucYear,stcTime.ucMonth,stcTime.ucDay,stcTime.ucHour,stcTime.ucMinute,(float)stcTime.ucSecond+(float)stcTime.usMiliSecond/1000);
//			delay_ms(10);
		//������ٶ�
		//���ڽ��ܵ��������Ѿ���������Ӧ�Ľṹ��ı������ˣ�����˵�����Э�飬�Լ��ٶ�Ϊ�� stcAcc.a[0]/32768*16����X��ļ��ٶȣ�
//		printf("Acc:%.3f %.3f %.3f\r\n",(float)stcAcc.a[0]/32768*16,(float)stcAcc.a[1]/32768*16,(float)stcAcc.a[2]/32768*16);
			delay_ms(10);
		//������ٶ�
//		printf("Gyro:%.3f %.3f %.3f\r\n",(float)stcGyro.w[0]/32768*2000,(float)stcGyro.w[1]/32768*2000,(float)stcGyro.w[2]/32768*2000);
//			delay_ms(10);
		//����Ƕ�
//		Angle_x= (float)stcAngle.Angle[0]/32768*180;
		Angle_z= (float)stcAngle.Angle[2]/32768*180;
//		printf("Angle:%.3f %.3f %.3f\r\n",(float)stcAngle.Angle[0]/32768*180,(float)stcAngle.Angle[1]/32768*180,(float)stcAngle.Angle[2]/32768*180);
//		if(Angle_z<0){
//			Angle_z=360+Angle_z;
//		}
		printf("angle_z:%.3f Gyro_z:%.3f",Angle_z,Gyro_z);
//		if(setpoint==0){
//			Angle_z_0=Angle_z;
//		}

//		Angle_z=Angle_z-Angle_z_0;
//		printf("angle_z:%.3f Gyro_z:%.3f",Angle_z,Gyro_z);
		Double_Roll_Pitch_PID_Calculation_z(&piddouble,setpoint, Angle_z, Gyro_z);
		
		
//	    if(error1<-3){
//				Motor3_rot(0);
//			}
//			else if(error1>3){
//				Motor3_rot(1);
//			}
//		if(piddouble.in.pid_result<0){
//			piddouble.in.pid_result=-piddouble.in.pid_result;
//		}
//	  printf("pwm:%.3f",150-piddouble.in.pid_result);
		//TIM_SetCompare1(TIM14,150-piddouble.in.pid_result);
	}
}
		


}

