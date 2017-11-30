#include "ekf.h"

// State vector
const float x0_f32_a[7] =
{
	1.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0
};

float Q_f32_a[6][6] = {{0},{0}};			// Process noise covariance
float R_f32_a[6][6] = {{0},{0}};			// Measurement noise covariance
float x_f32_a[7] = {0};					// State vector
float P_f32_a[7][7] = {{0},{0}};			// Error covariance
float ywf_f32[6] = {0};					// World frame origin measurement reference

void q2R(float R[3][3], float q[4]);
void computeF(float F[7][7], float Ts, float x[7], float u[3]);
void propagateState(float x[7], float u[3], float Ts);
void computeW(float W[7][6], float Ts, float x[7]);
void propagateCovariance(float P[7][7], float Ts, float gyro[3], float Q[6][6]);
void updateStateAndCovariance(float x[7], float P[7][7], float R[6][6], float y[6]);
void computeH(float H[6][7], float x[7], float y[6]);
int computeKalmanGain(float K[7][6], float H[6][7], float P[7][7], float R[6][6]);
void computeInnovation(float x[7], float y[6], float ywf_f32[6], float inno[6]);
void updateState(float x[7], float K[7][6], float innovation[6]);
void updateCovariance(float P[7][7], float K[7][6], float H[6][7], float R[6][6]);

// initialise variables
void init_ekf(imu_data_s * imu_data){
	int i;
	// initial covariance matrix
	for (i = 0; i < 7; i++)
		P_f32_a[i][i] = 0.1;

	// initial state
	for (i = 0; i < 7; i++)
		x_f32_a[i] = x0_f32_a[i];

	// process noise - gyro and bias
	for (i = 0; i < 3; i++)
		Q_f32_a[i][i] = ARDRONE_GYRO_VARIANCE;
	for (i = 3; i < 6; i++)
		Q_f32_a[i][i] = ARDRONE_BIAS_VARIANCE;

	// accel measurement noise
	for (i = 0; i < 3; i++)
		R_f32_a[i][i] = ARDRONE_ACCEL_VARIANCE;
	// magnetometer noise
	for (i = 3; i < 6; i++)
		R_f32_a[i][i] = ARDRONE_MAG_VARIANCE;

	for (i = 0; i < 3; i++){
		ywf_f32[i] = (float)imu_data->accel_offset[i];
		ywf_f32[i+3] = (float)imu_data->mag_offset[i];
		x_f32_a[i+4] = (float)imu_data->gyro_offset[i];
	}
}

void run_ekf(imu_data_s * imu_data){
	int i;
	float y[6];
	float gyro32[3];

	for (i = 0; i < 3; i++){
		y[i] = imu_data->acceleration[i];
		y[i+3] = imu_data->magnetic[i];
		gyro32[i] = imu_data->rate[i];
	}

	// propagate state
	propagateState(x_f32_a, gyro32, imu_data->Ts);
	// propagate covariance
	propagateCovariance(P_f32_a, imu_data->Ts, gyro32, Q_f32_a);
	// update phase
	updateStateAndCovariance(x_f32_a, P_f32_a, R_f32_a, y);

	normalise32(&x_f32_a[0], 4);
	memcpy(&imu_data->q[0], &x_f32_a[0], sizeof(float)*4);
	memcpy(&imu_data->w_bias[0], &x_f32_a[4], sizeof(float)*3);
}

void propagateState(float x[7], float u[3], float Ts){
	static float q_dot_prev[2][4] = {{0, 0, 0, 0},{0, 0, 0, 0}};
	int i;
	float q[4];			// quaternion state
	float b[3];			// bias state
	float w[3];			// rate
	float q_dot[4];		// current q_dot
	float q_next[4];	// next q_dot

	// copy state variables
	memcpy(&q[0], &x_f32_a[0], sizeof(float)*4);
	memcpy(&b[0], &x_f32_a[4], sizeof(float)*3);

	// w = u - b
	sub32v(u, b, w, 3);

	// compute qdot
	q_dot[0] = 0.5*(-q[1]*w[0] + -q[2]*w[1] + -q[3]*w[2]);
	q_dot[1] = 0.5*( q[0]*w[0] + -q[3]*w[1] +  q[2]*w[2]);
	q_dot[2] = 0.5*( q[3]*w[0] +  q[0]*w[1] + -q[1]*w[2]);
	q_dot[3] = 0.5*(-q[2]*w[0] +  q[1]*w[1] +  q[0]*w[2]);

	// integrate to find q
	q_next[0] = q[0] + (Ts/6.0)*(q_dot[0] + 4.0*q_dot_prev[0][0] + q_dot_prev[1][0]);
	q_next[1] = q[1] + (Ts/6.0)*(q_dot[1] + 4.0*q_dot_prev[0][1] + q_dot_prev[1][1]);
	q_next[2] = q[2] + (Ts/6.0)*(q_dot[2] + 4.0*q_dot_prev[0][2] + q_dot_prev[1][2]);
	q_next[3] = q[3] + (Ts/6.0)*(q_dot[3] + 4.0*q_dot_prev[0][3] + q_dot_prev[1][3]);

	// shift q_dot and store
	for (i = 0; i < 4; i++){
		q_dot_prev[1][i] = q_dot_prev[0][i];
		q_dot_prev[0][i] = q_dot[i];
	}

	// normalise
	normalise32(&q_next[0], 4);
	memcpy(&x[0], &q_next[0], 4*sizeof(float));	// bias remains the same
}

void computeF(float F[7][7], float Ts, float x[7], float u[3]){
	int i, j;
	float w[3];
	float q[4];

	// initialise F
	for (i = 0; i < 7; i++){
		for (j = 0; j < 7; j++){
			F[i][j] = 0.0;
		}
	}

	// calculate rate
	w[0] = u[0] - x[4];
	w[1] = u[1] - x[5];
	w[2] = u[2] - x[6];

	memcpy(&q[0], &x[0], 4*sizeof(float));

	// F row 1
	F[0][0] = Ts*0.5*0.0;
	F[0][1] = Ts*0.5*-w[0];
	F[0][2] = Ts*0.5*-w[1];
	F[0][3] = Ts*0.5*-w[2];
	F[0][4] = Ts*0.5*q[1];
	F[0][5] = Ts*0.5*q[2];
	F[0][6] = Ts*0.5*q[3];

	// F row 2
	F[1][0] = Ts*0.5*w[0];
	F[1][1] = Ts*0.5*0.0;
	F[1][2] = Ts*0.5*w[2];
	F[1][3] = Ts*0.5*-w[1];
	F[1][4] = Ts*0.5*-q[0];
	F[1][5] = Ts*0.5*q[3];
	F[1][6] = Ts*0.5*-q[2];

	// F row 3
	F[2][0] = Ts*0.5*w[1];
	F[2][1] = Ts*0.5*-w[2];
	F[2][2] = Ts*0.5*0.0;
	F[2][3] = Ts*0.5*w[0];
	F[2][4] = Ts*0.5*-q[3];
	F[2][5] = Ts*0.5*-q[0];
	F[2][6] = Ts*0.5*q[1];

	// F row 4
	F[3][0] = Ts*0.5*w[2];
	F[3][1] = Ts*0.5*w[1];
	F[3][2] = Ts*0.5*-w[0];
	F[3][3] = Ts*0.5*0.0;
	F[3][4] = Ts*0.5*q[2];
	F[3][5] = Ts*0.5*-q[1];
	F[3][6] = Ts*0.5*-q[0];

	// rest of the rows are zero

	// add identity to F
	for (i = 0; i < 7; i++)
		F[i][i] += 1;
}

void computeW(float W[7][6], float Ts, float x[7]){
	int i, j;
	float q[4];

	memcpy(&q[0], &x[0], 4*sizeof(float));

	// initialise W
	for (i = 0; i < 7; i++){
		for (j = 0; j < 6; j++){
			W[i][j] = 0.0;
		}
	}

	W[0][0] = Ts*0.5*-q[1];
	W[0][1] = Ts*0.5*-q[2];
	W[0][2] = Ts*0.5*-q[3];
	// W[0][3] ... W[0][5] = 0;

	W[1][0] = Ts*0.5*q[0];
	W[1][1] = Ts*0.5*-q[3];
	W[1][2] = Ts*0.5*q[2];
	// W[1][3] ... W[1][5] = 0;

	W[2][0] = Ts*0.5*q[3];
	W[2][1] = Ts*0.5*q[0];
	W[2][2] = Ts*0.5*-q[1];
	// W[2][3] ... W[2][5] = 0;

	W[3][0] = Ts*0.5*-q[2];
	W[3][1] = Ts*0.5*q[1];
	W[3][2] = Ts*0.5*q[0];
	// W[3][3] ... W[3][5] = 0;

	// W[4][0] ... W[4][2] = 0;
	W[4][3] = 1.0;
	// W[4][4] ... W[3][5] = 0;

	// W[5][0] ... W[5][3] = 0;
	W[5][4] = 1.0;
	// W[5][4] ... W[5][5] = 0;

	// W[6][0] ... W[6][4] = 0;
	W[6][5] = 1.0;
}

void propagateCovariance(float P[7][7], float Ts, float gyro[3], float Q[6][6]){
	float F[7][7] = {{0},{0}};
	float Ft[7][7] = {{0},{0}};
	float W[7][6] = {{0},{0}};
	float Wt[6][7] = {{0},{0}};
	float FP[7][7] = {{0},{0}};
	float FPFt[7][7] = {{0},{0}};
	float WQ[7][6] = {{0},{0}};
	float WQWt[7][7] = {{0},{0}};

	computeF(F, Ts, x_f32_a, gyro);
	computeW(W, Ts, x_f32_a);

	transpose32(&F[0][0], &Ft[0][0], 7, 7);
	mul32m(&F[0][0], &P[0][0], &FP[0][0], 7, 7, 7);
	mul32m(&FP[0][0], &Ft[0][0], &FPFt[0][0], 7, 7, 7);

	transpose32(&W[0][0], &Wt[0][0], 7, 6);
	mul32m(&W[0][0], &Q[0][0], &WQ[0][0], 7, 6, 6);
	mul32m(&WQ[0][0], &Wt[0][0], &WQWt[0][0], 7, 6, 7);

	add32m(&FPFt[0][0], &WQWt[0][0], &P[0][0], 7, 7);
}

void updateStateAndCovariance(float x[7], float P[7][7], float R[6][6], float y[6]){
	float H[6][7] = {{0},{0}};
	float K[7][6] = {{0},{0}};
	float innovation[6] = {0};

	// compute H
	computeH(H, x, ywf_f32);
	// compute Kalman gain
	computeKalmanGain(K, H, P, R);
	// calculate innovation
	computeInnovation(x, y, ywf_f32, innovation);
	// correct state
	updateState(x, K, innovation);
	// update covariance
	updateCovariance(P, K, H, R);
}

void computeH(float H[6][7], float x[7], float y[6]){
	float q[4];
	copy32v(&q[0], &x[0], 4);

	float dR_dq0[3][3] = {{2.0 *  q[0], 2.0 *  q[3],2.0 * -q[2]},
							  {2.0 * -q[3], 2.0 *  q[0],2.0 *  q[1]},
							  {2.0 *  q[2], 2.0 * -q[1],2.0 *  q[0]}};

	float dR_dq1[3][3] = {{2.0 *  q[1], 2.0 *  q[2],2.0 *  q[3]},
							  {2.0 *  q[2], 2.0 * -q[1],2.0 *  q[0]},
							  {2.0 *  q[3], 2.0 * -q[0],2.0 * -q[1]}};

	float dR_dq2[3][3] = {{2.0 * -q[2], 2.0 *  q[1],2.0 * -q[0]},
							  {2.0 *  q[1], 2.0 *  q[2],2.0 *  q[3]},
							  {2.0 *  q[0], 2.0 *  q[3],2.0 * -q[2]}};

	float dR_dq3[3][3] = {{2.0 * -q[3], 2.0 *  q[0],2.0 *  q[1]},
							  {2.0 * -q[0], 2.0 * -q[3],2.0 *  q[2]},
							  {2.0 *  q[1], 2.0 *  q[2],2.0 *  q[3]}};

	int i,j;
	// solve for column 0
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			H[i][0] += dR_dq0[i][j]*y[j];
			H[i+3][0] += dR_dq0[i][j]*y[j+3];
		}
	}
	// solve for column 1
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			H[i][1] += dR_dq1[i][j]*y[j];
			H[i+3][1] += dR_dq1[i][j]*y[j+3];
		}
	}
	// solve for column 2
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			H[i][2] += dR_dq2[i][j]*y[j];
			H[i+3][2] += dR_dq2[i][j]*y[j+3];
		}
	}
	// solve for column 3
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			H[i][3] += dR_dq3[i][j]*y[j];
			H[i+3][3] += dR_dq3[i][j]*y[j+3];
		}
	}
	// rest are zeros
}

int computeKalmanGain(float K[7][6], float H[6][7], float P[7][7], float R[6][6]){
	// K = P*H'/(H*P*H' + R)
	float Ht[7][6];
	float PHt[7][6];
	float HPHt[6][6];
	float HPHtpR[6][6];
	float K_den[6][6];
	float p[6][6];

	// transpose H
	transpose32(&H[0][0], &Ht[0][0], 6, 7);
	// matrix PHt
	mul32m(&P[0][0], &Ht[0][0], &PHt[0][0], 7, 7, 6);
	// matrix HPHt
	mul32m(&H[0][0], &PHt[0][0], &HPHt[0][0], 6, 7, 6);
	// K = PHt/(HPHt + R)
	add32m(&HPHt[0][0], &R[0][0], &HPHtpR[0][0], 6, 6);
	// K = PHt\(HPHt)
	if (cholsl(&HPHtpR[0][0], &K_den[0][0], &p[0][0], 6)) return 1;
	// K = PHt*K_den
	mul32m(&PHt[0][0], &K_den[0][0], &K[0][0], 7, 6, 6);
	return 0;
}

void computeInnovation(float x[7], float y[6], float ywf_f32[6], float inno[6]){
	float Rw2b[3][3];
	float q[4];
	copy32v(&q[0], &x[0], 4);
	q2R(Rw2b, q);

	float y_hat[6] = {0};
	int i,j;
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			y_hat[i] += Rw2b[i][j]*ywf_f32[j];
			y_hat[i+3] += Rw2b[i][j]*ywf_f32[j+3];
		}
	}

	sub32v(&y[0], &y_hat[0], &inno[0], 6);
}

void updateState(float x[7], float K[7][6], float innovation[6]){
	float Ke[7] = {0};
	float x_cpy[7] = {0};
	copy32v(x_cpy, x, 7);

	int i,j;
	for (i = 0; i < 7; i++){
		for (j = 0; j < 6; j++){
			Ke[i] += K[i][j]*innovation[j];
		}
	}

	add32v(&x_cpy[0], &Ke[0], &x[0], 7);
}

void updateCovariance(float P[7][7], float K[7][6], float H[6][7], float R[6][6]){
	float KH[7][7] = {{0},{0}};
	float I[7][7] = {{0},{0}};
	float IsKH[7][7] = {{0},{0}};
	float IsKHt[7][7] = {{0},{0}};
	float IsKHP[7][7] = {{0},{0}};
	float IsKHPIsKHt[7][7] = {{0},{0}};
	float Kt[6][7] = {{0},{0}};
	float KR[7][6] = {{0},{0}};
	float KRKt[7][7] = {{0},{0}};

	int i;
	for (i = 0; i < 7; i++)
		I[i][i] = 1;

	// P = (I - K*H)*P*(I - K*H)' + K*EKF.R*K';
	mul32m(&K[0][0], &H[0][0], &KH[0][0], 7, 6, 7);
	sub32m(&I[0][0], &KH[0][0], &IsKH[0][0], 7, 7);
	// P = IsKH*P*IsKH' + K*EKF.R*K';
	transpose32(&IsKH[0][0], &IsKHt[0][0], 7, 7);
	// P = IsKH*P*IsKHt + K*EKF.R*K';
	mul32m(&IsKH[0][0], &P[0][0], &IsKHP[0][0], 7, 7, 7);
	// P = IsKHP*IsKHt + K*EKF.R*K';
	mul32m(&IsKHP[0][0], &IsKHt[0][0], &IsKHPIsKHt[0][0], 7, 7, 7);
	// P = IsKHPIsKHt + K*EKF.R*K';
	transpose32(&K[0][0], &Kt[0][0], 7, 6);
	// P = IsKHPIsKHt + K*EKF.R*Kt;
	mul32m(&K[0][0], &R[0][0], &KR[0][0], 7, 6, 6);
	// P = IsKHPIsKHt + KR*Kt;
	mul32m(&KR[0][0], &Kt[0][0], &KRKt[0][0], 7, 6, 7);
	// P = IsKHPIsKHt + KRKt;
	add32m(&IsKHPIsKHt[0][0], &KRKt[0][0], &P[0][0], 7, 7);
}

void q2R(float R[3][3], float q[4]){
	R[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
	R[0][1] = 2.0*(q[1]*q[2] + q[0]*q[3]);
	R[0][2] = 2.0*(q[1]*q[3] - q[0]*q[2]);

	R[1][0] = 2.0*(q[1]*q[2] - q[0]*q[3]);
	R[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
	R[1][2] = 2.0*(q[2]*q[3] + q[0]*q[1]);

	R[2][0] = 2.0*(q[1]*q[3] + q[0]*q[2]);
	R[2][1] = 2.0*(q[2]*q[3] - q[0]*q[1]);
	R[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
}

void q2ypr(float q[4], float ypr[3]){
	ypr[0] = atan2(2.0*q[2]*q[3] + 2.0*q[0]*q[1], q[3]*q[3] - q[2]*q[2] - q[1]*q[1] + q[0]*q[0])*180.0/M_PI;
	ypr[1] = -asin(2.0*q[1]*q[3] - 2.0*q[0]*q[2])*180.0/M_PI;
	ypr[2] = atan2(2.0*q[1]*q[2] + 2.0*q[0]*q[3], q[1]*q[1] + q[0]*q[0] - q[3]*q[3] - q[2]*q[2])*180.0/M_PI;
}
