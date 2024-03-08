/*
 * mainpp.h
 *
 *  Created on: 2018/01/17
 *      Author: yoneken
 */
#include <stdbool.h>
#include <main.h>
#ifndef MAINPP_H_
#define MAINPP_H_

#ifdef __cplusplus
extern "C" {
#endif

void Rosserial_Init(void);
void Rosserial_Spin(void);
bool Rosserial_Checkconfigstate(void);
void Rosserial_GetHardware(void);
void odom_store(void);
void odom_publish(void);

extern double odom_vel[3];

#ifdef __cplusplus
}
#endif


#endif /* MAINPP_H_ */
