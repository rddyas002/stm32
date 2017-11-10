/*
 * ekf.h
 *
 *  Created on: 09 Nov 2017
 *      Author: yreddi
 */

#ifndef EKF_H_
#define EKF_H_

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "arm_math.h"

void init_ekf(float32_t ywf[6]);
void run_ekf(float Ts, float gyro[3], float accel[3], float magnetic[3], float * q, float * w);

#endif /* EKF_H_ */
