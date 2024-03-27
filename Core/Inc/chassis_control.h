/*
 * control.h
 *
 *  Created on: Feb 1, 2024
 *      Author: Jing Chen
 */
#ifndef INC_CHASSIS_CONTROL_H_
#define INC_CHASSIS_CONTROL_H_

#include "main.h"

#define ML_DIR_Pin GPIOB
#define ML_DIR_GPIO_port GPIO_PIN_13
#define MR_DIR_Pin GPIOD
#define MR_DIR_GPIO_port GPIO_PIN_8

// encoder and pwm timer
#define ML_Encoder_timer htim3
#define ML_Encoder_timerchannel TIM_CHANNEL_1 | TIM_CHANNEL_2
#define ML_Encoder_dir -1
#define ML_PWM_timer htim12
#define ML_PWM_timerchannel TIM_CHANNEL_2

#define MR_Encoder_timer htim4
#define MR_Encoder_timerchannel TIM_CHANNEL_1 | TIM_CHANNEL_2
#define MR_Encoder_dir -1
#define MR_PWM_timer htim12
#define MR_PWM_timerchannel TIM_CHANNEL_1

#define ML_KP 2
#define ML_KI 40
#define ML_KD 0.0

#define MR_KP 2
#define MR_KI 50
#define MR_KD 0.0

#define Encoder_Interrupt_timer htim23
//#define control_period 0.01 // frequency = 84M/(13+1)/(60000) = 100HZ
extern double control_period;

/* Hardware Info */
#define encoder_resolution 1024
#define speed_reduction_ratio 44 // maxon: 44

#define wheel_radius 0.078
#define chassis_radius 0.13

extern double radius_error_l;
extern double radius_error_r;
extern double radius_error_chassis;

extern double linearvelocity_x;
extern double linearvelocity_y;
extern double angularvelocity;


// PID control param define
extern double limit_integral;
extern double pwm_arr;

typedef struct
{
		double Kp, Ki, Kd;

		int16_t CountNum;
		double rps, rps_before;
		double goal, err;
		double propotional, integral, differential;
		double duty;

		GPIO_TypeDef *DIR_pin_type;
		uint16_t DIR_pin_Num;
		TIM_HandleTypeDef encoder_timer;
		uint32_t encoder_timer_channel;
		int encoder_dir;
		TIM_HandleTypeDef pwm_timer;
		uint32_t pwm_timer_channel;

}PID_Control;

extern PID_Control Wheel_L;
extern PID_Control Wheel_R;

extern int i;
extern double sssss[1000];

void Control_Init();
void Pid_Param_Init(PID_Control *Wheel_, double kp, double ki, double kd);
void Hardware_Info_Init();
void Control_Timer_Init();
void PID_Controller(PID_Control *Wheel_);
void Forward_Kinematics(double x, double y, double w);
void Inverse_Kinematics(PID_Control *WheelL_, PID_Control *WheelR_);
void Stop_Chasis();

void Motor_Driver_Init(PID_Control *Wheel_,
		GPIO_TypeDef *DIR_pin_type, uint16_t DIR_pin_num_,
		TIM_HandleTypeDef encoder_timer_, uint32_t encoder_timer_channel_, int encoder_dir_,
		TIM_HandleTypeDef pwm_timer_, uint32_t pwm_timer_channel_);

#endif /* INC_CHASSIS_CONTROL_H_ */
