/*
 * led_control.h
 *
 *  Created on: Mar 8, 2024
 *      Author: user
 */

#ifndef INC_LED_CONTROL_H_
#define INC_LED_CONTROL_H_

#include "main.h"

#define LED1_DIR_Pin GPIOE
#define LED1_DIR_GPIO_port GPIO_PIN_3

#define LED2_DIR_Pin GPIOC
#define LED2_DIR_GPIO_port GPIO_PIN_13

extern int LED_state;

void LED_Init();
void LED_Control(int led_state);

#endif /* INC_LED_CONTROL_H_ */
