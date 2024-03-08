/*
 * led_control.c
 *
 *  Created on: Mar 8, 2024
 *      Author: user
 */

#include <led_control.h>

int LED_state;
void LED_Init()
{
	LED_state = 0;
	HAL_GPIO_WritePin(LED1_DIR_Pin, LED1_DIR_GPIO_port, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_DIR_Pin, LED2_DIR_GPIO_port, GPIO_PIN_RESET);
}

void LED_Control(int led_state)
{
	switch(led_state)
	{
		case 10:
			HAL_GPIO_WritePin(LED1_DIR_Pin, LED1_DIR_GPIO_port, GPIO_PIN_RESET);
			break;
		case 11:
			HAL_GPIO_WritePin(LED1_DIR_Pin, LED1_DIR_GPIO_port, GPIO_PIN_SET);
			break;
		case 20:
			HAL_GPIO_WritePin(LED2_DIR_Pin, LED2_DIR_GPIO_port, GPIO_PIN_RESET);
			break;
		case 21:
			HAL_GPIO_WritePin(LED2_DIR_Pin, LED2_DIR_GPIO_port, GPIO_PIN_SET);
			break;
		default:
			break;
	}
	LED_state = 0;
}
