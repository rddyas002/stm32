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

#include "matrix_math.h"

#define M_PI_f		3.14159265358979323846f

typedef struct{
	float rate[3];			// dps
	float acceleration[3];	// g
	float magnetic[3];		// uT
	float gyro_var[3];
	float accel_var[3];
	float mag_var[3];
	float gyro_offset[3];
	float accel_offset[3];
	float mag_offset[3];
	float time;
}__attribute__((packed)) imu_data_s;

void init_ekf(imu_data_s * imu_data);
void run_ekf(double Ts, float gyro[3], float accel[3], float magnetic[3], double * q, double * w);
void q2ypr(double q[4], double ypr[3]);
void triadComputation(float r1_wf[3], float r2_wf[3], float r1_bf[3], float r2_bf[3], float ypr[3]);

#endif /* EKF_H_ */
