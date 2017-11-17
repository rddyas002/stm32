/*
 * ekf.h
 *
 *  Created on: 09 Nov 2017
 *      Author: yreddi
 */

#ifndef EKF32_H_
#define EKF32_H_

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "arm_math.h"

#include "ekf.h"

void init_ekf_32(imu_data_s * imu_data);
void run_ekf_32(float Ts, float gyro[3], float accel[3], float magnetic[3], float * q, float * w);
void q2ypr_32(float32_t q[4], float32_t ypr[3]);
void triadComputation_32(float32_t r1_wf[3], float32_t r2_wf[3], float32_t r1_bf[3], float32_t r2_bf[3], float32_t ypr[3]);

#endif /* EKF32_H_ */
