/*
 * control.c
 *
 *  Created on: Feb 1, 2024
 *      Author: Jing Chen
*/

#include <chassis_control.h>
#include <math.h>

double radius_error_l;
double radius_error_r;
double radius_error_chassis;

double linearvelocity_x;
double linearvelocity_y;
double angularvelocity;

double limit_integral;
double pwm_arr;
double control_period;

PID_Control Wheel_L = {0};
PID_Control Wheel_R = {0};

int i;
double sssss[1000] = {0};

/**
 * @ brief Include all the initial function
 * @ retval None
 * */
void Control_Init()
{
	Hardware_Info_Init();
	Control_Timer_Init();

#ifdef G2_18V17
	Motor_Driver_Init(&Wheel_L, M1_DIR_Pin, M1_DIR_GPIO_port, M1_Encoder_timer, M1_Encoder_timerchannel, M1_Encoder_dir, M1_PWM_timer, M1_PWM_timerchannel);
	Motor_Driver_Init(&Wheel_R, M2_DIR_Pin, M2_DIR_GPIO_port, M2_Encoder_timer, M2_Encoder_timerchannel, M2_Encoder_dir, M2_PWM_timer, M2_PWM_timerchannel);
#endif

	Pid_Param_Init(&Wheel_L, M1_KP, M1_KI, M1_KD);
	Pid_Param_Init(&Wheel_R, M2_KP, M2_KI, M2_KD);

	i = 0;
	limit_integral = 1.0;
	pwm_arr = M1_PWM_timer.Init.Period;

	// PCLK1_freq, APB1 timer frequency
	int32_t PCLK1_freq = HAL_RCC_GetPCLK1Freq();

	if((RCC->CFGR & RCC_D2CFGR_D2PPRE1) != 0)
	{
		PCLK1_freq *=2;
	}

	int32_t timer_interrupt_freq = (double)PCLK1_freq / (Encoder_Interrupt_timer.Init.Prescaler + 1) / (Encoder_Interrupt_timer.Init.Period + 1);
	control_period = (double)(1 / (double)timer_interrupt_freq);

	Wheel_L.integral = 0.0;
	Wheel_R.integral = 0.0;

	Wheel_L.goal = 0.0;
	Wheel_R.goal = 0.0;

	// stop chassis
	HAL_GPIO_WritePin(Wheel_L.DIR_pin_type, Wheel_L.DIR_pin_Num, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&(Wheel_L.pwm_timer), Wheel_L.pwm_timer_channel, 0);

	HAL_GPIO_WritePin(Wheel_R.DIR_pin_type, Wheel_R.DIR_pin_Num, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&(Wheel_R.pwm_timer), Wheel_R.pwm_timer_channel, 0);
}


/**
 * @ brief assign the pid gain value into the PID_Controll object
 * @ retval None
 * */
void Pid_Param_Init(PID_Control *Wheel_, double kp, double ki, double kd)
{
	Wheel_->Kp = kp;
	Wheel_->Ki = ki;
	Wheel_->Kd = kd;
}


/**
 * @ brief assign the required param of each motors into the PID_Controll object
 * @ define choose the board used : VNH5019 or DRV8874, change the define in "control.h"
 * @ param Wheel_ the object declare for each motor (WheelA, WheelB, WheelC)
 * @ param all the other param is define in "control.h"
 * @ retval None
 * */
#ifdef VNH5019
void Motor_Driver_Init(PID_Control *Wheel_,
		GPIO_TypeDef *INA_pin_type_, uint16_t INA_pin_num_,
		GPIO_TypeDef *INB_pin_type_, uint16_t INB_pin_num_,
		TIM_HandleTypeDef encoder_timer_, uint32_t encoder_timer_channel_, int encoder_dir_,
		TIM_HandleTypeDef pwm_timer_, uint32_t pwm_timer_channel_)
{
	Wheel_->INA_pin_type = INA_pin_type_;
	Wheel_->INA_pin_Num = INA_pin_num_;
	Wheel_->INB_pin_type = INB_pin_type_;
	Wheel_->INB_pin_Num = INB_pin_num_;
	Wheel_->encoder_timer = encoder_timer_;
	Wheel_->encoder_timer_channel = encoder_timer_channel_;
	Wheel_->encoder_dir = encoder_dir_;
	Wheel_->pwm_timer = pwm_timer_;
	Wheel_->pwm_timer_channel = pwm_timer_channel_;
}
#endif
#ifdef DRV8874
void Motor_Driver_Init(PID_Control *Wheel_,
		GPIO_TypeDef *PHASE_pin_type_, uint16_t PHASE_pin_num_,
		TIM_HandleTypeDef encoder_timer_, uint32_t encoder_timer_channel_, int encoder_dir_,
		TIM_HandleTypeDef pwm_timer_, uint32_t pwm_timer_channel_)
{
	Wheel_->PHASE_pin_type = PHASE_pin_type_;
	Wheel_->PHASE_pin_Num = PHASE_pin_num_;
	Wheel_->encoder_timer = encoder_timer_;
	Wheel_->encoder_timer_channel = encoder_timer_channel_;
	Wheel_->encoder_dir = encoder_dir_;
	Wheel_->pwm_timer = pwm_timer_;
	Wheel_->pwm_timer_channel = pwm_timer_channel_;
}
#endif
#ifdef G2_18V17
void Motor_Driver_Init(PID_Control *Wheel_,
		GPIO_TypeDef *DIR_pin_type, uint16_t DIR_pin_num_,
		TIM_HandleTypeDef encoder_timer_, uint32_t encoder_timer_channel_, int encoder_dir_,
		TIM_HandleTypeDef pwm_timer_, uint32_t pwm_timer_channel_)
{
	Wheel_->DIR_pin_type = DIR_pin_type;
	Wheel_->DIR_pin_Num = DIR_pin_num_;
	Wheel_->encoder_timer = encoder_timer_;
	Wheel_->encoder_timer_channel = encoder_timer_channel_;
	Wheel_->encoder_dir = encoder_dir_;
	Wheel_->pwm_timer = pwm_timer_;
	Wheel_->pwm_timer_channel = pwm_timer_channel_;
}
#endif


/**
 * @ brief initialize the timers that encoder and pwm used
 * @ all the definition in the function are in "control.h"
 * @ retval None
 * */
void Control_Timer_Init()
{
	HAL_TIM_Encoder_Start(&M1_Encoder_timer, M1_Encoder_timerchannel);
	HAL_TIM_PWM_Start(&M1_PWM_timer, M1_PWM_timerchannel);
	HAL_TIM_Encoder_Start(&M2_Encoder_timer, M2_Encoder_timerchannel);
	HAL_TIM_PWM_Start(&M2_PWM_timer, M2_PWM_timerchannel);
}


/**
 * @ brief assign the hardware value for motor and chassis radius error
 * @ retval None
 * */
void Hardware_Info_Init()
{

	radius_error_l = 1.0;
	radius_error_r = 1.0;
	radius_error_chassis = 1.0;
}


/**
 * @ brief PID control for the motor
 * @ param Wheel_ the object declare for each motor (WheelA, WheelB, WheelC)
 * @ retval None
 * */
void PID_Controller(PID_Control *Wheel_)
{

	Wheel_->CountNum = __HAL_TIM_GetCounter(&Wheel_->encoder_timer)* Wheel_->encoder_dir;
	Wheel_->rps = (double)Wheel_->CountNum / ((double)4 * encoder_resolution * speed_reduction_ratio * control_period);
	__HAL_TIM_SetCounter(&Wheel_->encoder_timer ,0);

//	if (i < 700)
//	{
//		sssss[i++] = Wheel_->rps;
//	}

	Wheel_->err = Wheel_->goal - Wheel_->rps;
	Wheel_->propotional = (double)Wheel_->err * Wheel_->Kp;
	Wheel_->integral += (double)Wheel_->err * Wheel_->Ki * control_period;
	Wheel_->integral = (Wheel_->integral > limit_integral)? limit_integral : Wheel_->integral;
	Wheel_->integral = (Wheel_->integral < (double)(-1) * limit_integral)? (double)(-1) * limit_integral : Wheel_->integral;
	Wheel_->differential = (double) Wheel_->Kd * (-1) * (Wheel_->rps - Wheel_->rps_before) / control_period;

	Wheel_->duty = Wheel_->propotional + Wheel_->integral + Wheel_->differential;
	Wheel_->duty = (Wheel_->duty > 1)? 1 : Wheel_->duty;
	Wheel_->duty = (Wheel_->duty < -1)? -1 : Wheel_->duty;

#ifdef G2_18V17
	if(Wheel_->duty >= 0)
	{
		HAL_GPIO_WritePin(Wheel_->DIR_pin_type, Wheel_->DIR_pin_Num, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
	else
	{
		HAL_GPIO_WritePin(Wheel_->DIR_pin_type, Wheel_->DIR_pin_Num, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
#endif
#ifdef VNH5019
	if(Wheel_->duty >= 0)
	{
		HAL_GPIO_WritePin(Wheel_->INA_pin_type, Wheel_->INA_pin_Num, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Wheel_->INB_pin_type, Wheel_->INB_pin_Num, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
	else
	{
		HAL_GPIO_WritePin(Wheel_->INA_pin_type, Wheel_->INA_pin_Num, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Wheel_->INB_pin_type, Wheel_->INB_pin_Num, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
#endif
#ifdef DRV8874
	if(Wheel_->duty >= 0)
	{
		HAL_GPIO_WritePin(Wheel_->PHASE_pin_type, Wheel_->PHASE_pin_Num, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
	else
	{
		HAL_GPIO_WritePin(Wheel_->PHASE_pin_type, Wheel_->PHASE_pin_Num, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&(Wheel_->pwm_timer), Wheel_->pwm_timer_channel, fabs(Wheel_->duty * pwm_arr));
	}
#endif

	Wheel_->rps_before = Wheel_->rps;
}


/**
 * @ brief turn the chassis velocity into wheel angular velocity
 * @ param x linear velocity for the chassis in x-direction
 * @ param y linear velocity for the chassis in y-direction
 * @ param w angular velocity for the chassis (counter-clockwise is positive )
 *
 *          A                   x (+)
 *         / \                  ↑
 *        /   \                 |
 *       /     \        y(+) ←---
 *      B-------C
 * @ retval None
 * */
void Forward_Kinematics(double x, double y, double w)
{
	double omega_left = (x + w * chassis_radius * radius_error_chassis) / (wheel_radius * radius_error_l);
	double omega_right = (x - w * chassis_radius * radius_error_chassis) / (wheel_radius * radius_error_r);

	Wheel_L.goal = omega_left / (2 * M_PI);
	Wheel_R.goal = omega_right / (2 * M_PI);
}


void Inverse_Kinematics(PID_Control *WheelL_, PID_Control *WheelR_)
{
	double vl = WheelL_->rps * (2 * M_PI) * (wheel_radius * radius_error_l);
	double vr = WheelR_->rps * (2 * M_PI) * (wheel_radius * radius_error_r);

	double Vx = (vl + vr) / 2;
	double Vy = 0;
	double Vw = (vr - vl) / (chassis_radius * radius_error_chassis);

	odom_vel[0] = Vx;
	odom_vel[1] = Vy;
	odom_vel[2] = Vw;
	odom_store();
}

/**
 * @ brief set the input speed zero
 * @ retval None
 * */
void Stop_Chasis()
{
	linearvelocity_x = 0;
	linearvelocity_y = 0;
	angularvelocity = 0;
}

