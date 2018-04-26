/*
 * gpio.h
 *
 *  Created on: 06 Nov 2017
 *      Author: yreddi
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stm32f4xx.h>

#define PWM_MIN 	2296
#define PWM_ZERO 	2976
#define PWM_MAX		3664

void init_gpio(void);
void init_tim4(void);

#endif /* GPIO_H_ */
