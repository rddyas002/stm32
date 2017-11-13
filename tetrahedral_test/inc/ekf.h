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

#define M_PI_f		3.14159265358979323846f

void init_ekf(float32_t ywf[6]);
void run_ekf(float Ts, float gyro[3], float accel[3], float magnetic[3], float * q, float * w);
void q2ypr(float32_t q[4], float32_t ypr[3]);
void triadComputation(float32_t r1_wf[3], float32_t r2_wf[3], float32_t r1_bf[3], float32_t r2_bf[3], float32_t ypr[3]);

#endif /* EKF_H_ */
