//References:
// https://www.servotecnica.com/en/resources/white-papers-en-mobile/dual-loop-advanced-control-techniques-for-real-world-drivetrains/
// https://controlguru.com/the-cascade-control-architecture/

/**
 * PID control.c
 * @author ChrisP @ M-HIVE

 * This library source code is for cascade double loop pid control for STM32 Drone Development online course.
 *
 * Created by ChrisP(Wonyeob Park) @ M-HIVE Embedded Academy, July, 2020
 * Rev. 1.0
 *
 * Where to take the online course.
 * https://www.inflearn.com/course/STM32CubelDE-STM32F4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C (Korean language supported only)
 *
 * Where to buy MH-FC V2.2 STM32F4 Drone Flight Controller.
 * https://smartstore.naver.com/mhivestore/products/4961922335
 *
 * https://github.com/ChrisWonyeobPark
 * https://blog.naver.com/lbiith
 * https://cafe.naver.com/mhiveacademy
 * https://www.udemy.com/course/stm32-drone-programming/?referralCode=E24CB7B1CD9993855D45
 * https://www.inflearn.com/course/stm32cubelde-stm32f4%EB%93%9C%EB%A1%A0-%EA%B0%9C%EB%B0%9C
*/

#include "PID control2.h"
#include "motor.h"
#include "delay.h"

PIDDouble roll2;
PIDDouble pitch2;
PIDSingle yaw_heading2;
PIDSingle yaw_rate2;

#define DT 0.001f
#define OUTER_DERIV_FILT_ENABLE 1
#define INNER_DERIV_FILT_ENABLE 1

void Double_Roll_Pitch_PID_Calculation2(PIDDouble* axis, float set_point_angle, float angle/*BNO080 旋转角度*/, float rate/*ICM-20602 角速度*/)
{
	/*********** Double PID Outer Begin (滚动和俯仰角度位置控制) *************/
	axis->out.reference = set_point_angle;	//Set point of outer PID control外部PID控制量
	axis->out.meas_value = angle;			//BNO080 rotation angle旋转角度

	axis->out.error = axis->out.reference - axis->out.meas_value;	//Define error of outer loop误差
	axis->out.p_result = axis->out.error * axis->out.kp;			//计算外循环P结果（kp*error）
	


	axis->out.error_sum = axis->out.error_sum + axis->out.error * DT;	//Define summation of outer loop 定义外循环求和
#define OUT_ERR_SUM_MAX 500
#define OUT_I_ERR_MIN -OUT_ERR_SUM_MAX
	/*限幅*/
	if(axis->out.error_sum > OUT_ERR_SUM_MAX) axis->out.error_sum = OUT_ERR_SUM_MAX;
	else if(axis->out.error_sum < OUT_I_ERR_MIN) axis->out.error_sum = OUT_I_ERR_MIN;
	
	axis->out.i_result = axis->out.error_sum * axis->out.ki;			//Calculate I result of outer loop 积分误差

	axis->out.error_deriv = -rate;										//定义外环导数（rate = ICM-20602 角速率）

/*两种算法（是否滤波）*/
#if !OUTER_DERIV_FILT_ENABLE
	axis->out.d_result = axis->out.error_deriv * axis->out.kd;			//Calculate D result of outer loop
#else
	axis->out.error_deriv_filt = axis->out.error_deriv_filt * 0.4f + axis->out.error_deriv * 0.6f;	//filter for derivative
	axis->out.d_result = axis->out.error_deriv_filt * axis->out.kd;									//Calculate D result of inner loop
#endif

	axis->out.pid_result = axis->out.p_result + axis->out.i_result + axis->out.d_result;  //Calculate PID result of outer loop外环控制量
	/****************************************************************************************/
	
	/************ Double PID Inner Begin (Roll and Pitch Angular Rate Control) 双PID内部开始（滚动和俯仰角率控制**************/
	axis->in.reference = axis->out.pid_result;	//Set point of inner PID control is the PID result of outer loop (for double PID control)
	axis->in.meas_value = rate;					//ICM-20602 angular rate

	axis->in.error = axis->in.reference - axis->in.meas_value;	//Define error of inner loop
	axis->in.p_result = axis->in.error * axis->in.kp;			//Calculate P result of inner loop

	axis->in.error_sum = axis->in.error_sum + axis->in.error * DT;	//Define summation of inner loop
#define IN_ERR_SUM_MAX 500
#define IN_I_ERR_MIN -IN_ERR_SUM_MAX
	if(axis->out.error_sum > IN_ERR_SUM_MAX) axis->out.error_sum = IN_ERR_SUM_MAX;
	else if(axis->out.error_sum < IN_I_ERR_MIN) axis->out.error_sum = IN_I_ERR_MIN;
	axis->in.i_result = axis->in.error_sum * axis->in.ki;							//Calculate I result of inner loop

	axis->in.error_deriv = -(axis->in.meas_value - axis->in.meas_value_prev) / DT;	//Define derivative of inner loop
	axis->in.meas_value_prev = axis->in.meas_value;									//Refresh value_prev to the latest value

#if !INNER_DERIV_FILT_ENABLE
	axis->in.d_result = axis->in.error_deriv * axis->in.kd;				//Calculate D result of inner loop
#else
	axis->in.error_deriv_filt = axis->in.error_deriv_filt * 0.5f + axis->in.error_deriv * 0.5f;	//filter for derivative
	axis->in.d_result = axis->in.error_deriv_filt * axis->in.kd;								//Calculate D result of inner loop
#endif
	
	axis->in.pid_result = axis->in.p_result + axis->in.i_result + axis->in.d_result; //Calculate PID result of inner loop
	/****************************************************************************************/
}

void Single_Yaw_Heading_PID_Calculation2(PIDSingle* axis, float set_point_angle, float angle/*BNO080 Rotation Angle*/, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Position) *************/
	axis->reference = set_point_angle;	//Set point of yaw heading @ yaw stick is center.
	axis->meas_value = angle;			//Current BNO080_Yaw angle @ yaw stick is center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw angle control

	if(axis->error > 180.f) axis->error -= 360.f;
	else if(axis->error < -180.f) axis->error += 360.f;
	
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw angle control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw angle control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw angle control

	axis->error_deriv = -rate;						//Define differentiation of yaw angle control
	axis->d_result = axis->error_deriv * axis->kd;	//Calculate D result of yaw angle control
	
	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw angle control
	/***************************************************************/
}

void Single_Yaw_Rate_PID_Calculation2(PIDSingle* axis, float set_point_rate, float rate/*ICM-20602 Angular Rate*/)
{
	/*********** Single PID Begin (Yaw Angular Rate Control) *************/
	axis->reference = set_point_rate;	//Set point of yaw heading @ yaw stick is not center.
	axis->meas_value = rate;			//Current ICM20602.gyro_z @ yaw stick is not center.

	axis->error = axis->reference - axis->meas_value;	//Define error of yaw rate control
	axis->p_result = axis->error * axis->kp;			//Calculate P result of yaw rate control

	axis->error_sum = axis->error_sum + axis->error * DT;	//Define summation of yaw rate control
	axis->i_result = axis->error_sum * axis->ki;			//Calculate I result of yaw rate control

	axis->error_deriv = -(axis->meas_value - axis->meas_value_prev) / DT;	//Define differentiation of yaw rate control
	axis->meas_value_prev = axis->meas_value;								//Refresh value_prev to the latest value
	axis->d_result = axis->error_deriv * axis->kd;							//Calculate D result of yaw rate control

	axis->pid_result = axis->p_result + axis->i_result + axis->d_result; //Calculate PID result of yaw control
	/*******************************************************************/
}

void Reset_PID_Integrator2(PIDSingle* axis)
{
	axis->error_sum = 0;
}

void Reset_All_PID_Integrator2(void)
{
	Reset_PID_Integrator2(&roll2.in);
	Reset_PID_Integrator2(&roll2.out);
	Reset_PID_Integrator2(&pitch2.in);
	Reset_PID_Integrator2(&pitch2.out);
	Reset_PID_Integrator2(&yaw_heading2);
	Reset_PID_Integrator2(&yaw_rate2);
}
