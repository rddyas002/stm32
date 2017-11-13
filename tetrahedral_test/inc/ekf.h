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

typedef struct{
	float32_t rate[3];			// dps
	float32_t acceleration[3];	// g
	float32_t magnetic[3];		// uT
	float32_t gyro_var[3];
	float32_t accel_var[3];
	float32_t mag_var[3];
	float32_t gyro_offset[3];
	float32_t accel_offset[3];
	float32_t mag_offset[3];
	float32_t time;
}__attribute__((packed)) imu_data_s;

#define M_PI_f		3.14159265358979323846f

void init_ekf(imu_data_s * imu_data);
void run_ekf(float32_t Ts, float32_t gyro[3], float32_t accel[3], float32_t magnetic[3], float32_t * q, float32_t * w);
void q2ypr(float32_t q[4], float32_t ypr[3]);
void triadComputation(float32_t r1_wf[3], float32_t r2_wf[3], float32_t r1_bf[3], float32_t r2_bf[3], float32_t ypr[3]);

#endif /* EKF_H_ */
