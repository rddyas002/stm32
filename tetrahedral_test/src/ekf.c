#include "ekf.h"
//#define ARM
// State error covariance matrix
const double P0_f32_a[7][7] =
{
	{1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
	{0.0, 1, 0.0, 0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 1, 0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 1, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0, 1, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0, 0.0, 1, 0.0},
	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1}
};

// State vector
const double x0_f32_a[7] =
{
	1.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0
};

double Q_f64_a[6][6] = {{0},{0}};			// Process noise covariance
double R_f64_a[6][6] = {{0},{0}};			// Measurement noise covariance
double x_f64_a[7] = {0};					// State vector
double P_f64_a[7][7] = {{0},{0}};			// Error covariance
double ywf_f64[6] = {0};					// World frame origin measurement reference

void q2R(double R[3][3], double q[4]);
void computeF(double F[7][7], double Ts, double x[7], double u[3]);
void propagateState(double x[7], double u[3], double Ts);
void computeW(double W[7][6], double Ts, double x[7]);
void propagateCovariance(double P[7][7], double Ts, double gyro[3], double Q[6][6]);
void updateStateAndCovariance(double x[7], double P[7][7], double R[6][6], double y[6]);
void computeH(double H[6][7], double x[7], double y[6]);
int computeKalmanGain(double K[7][6], double H[6][7], double P[7][7], double R[6][6]);
void computeInnovation(double x[7], double y[6], double ywf_f32[6], double inno[6]);
void updateState(double x[7], double K[7][6], double innovation[6]);
void updateCovariance(double P[7][7], double K[7][6], double H[6][7], double R[6][6]);

// initialise variables
void init_ekf(imu_data_s * imu_data){
	int i;
	// initial covariance matrix
	for (i = 0; i < 7; i++)
		P_f64_a[i][i] = 0.1;

	// initial state
	for (i = 0; i < 7; i++)
		x_f64_a[i] = x0_f32_a[i];

	// process noise - gyro and bias
	for (i = 0; i < 3; i++)
		Q_f64_a[i][i] = 0.35e-5;
	for (i = 3; i < 6; i++)
		Q_f64_a[i][i] = 2.5e-8;

	// accel measurement noise
	for (i = 0; i < 3; i++)
		R_f64_a[i][i] = 0.3e-4;
	// magnetometer noise
	for (i = 3; i < 6; i++)
		R_f64_a[i][i] = 0.4e-4;

	for (i = 0; i < 3; i++){
		ywf_f64[i] = (double)imu_data->accel_offset[i];
		ywf_f64[i+3] = (double)imu_data->mag_offset[i];
		x_f64_a[i+4] = (double)imu_data->gyro_offset[i];
	}
/*
	ywf_f64[0] = 0.0039;
	ywf_f64[1] = 0.0173;
	ywf_f64[2] = -1;
	ywf_f64[3] = 0.1224;
	ywf_f64[4] = -0.6577;
	ywf_f64[5] = 0.7433;
	x_f64_a[0] = 1;
	x_f64_a[1] = 0;
	x_f64_a[2] = 0;
	x_f64_a[3] = 0;
	x_f64_a[4] = 0;
	x_f64_a[5] = 0;
	x_f64_a[6] = 0;
*/
}

void run_ekf(double Ts, float gyro[3], float accel[3], float magnetic[3], double * q, double * w){
	int i;
	double y[6];
	double gyro64[3];

	for (i = 0; i < 3; i++){
		y[i] = (double)accel[i];
		y[i+3] = (double)magnetic[i];
		gyro64[i] = (double)gyro[i];
	}

	// propagate state
	propagateState(x_f64_a, gyro64, Ts);
	// propagate covariance
	propagateCovariance(P_f64_a, Ts, gyro64, Q_f64_a);
	// update phase
	updateStateAndCovariance(x_f64_a, P_f64_a, R_f64_a, y);

	memcpy(q, &x_f64_a[0], sizeof(double)*4);
	memcpy(w, &x_f64_a[4], sizeof(double)*3);
}

void propagateState(double x[7], double u[3], double Ts){
	static double q_dot_prev[2][4] = {{0, 0, 0, 0},{0, 0, 0, 0}};
	int i;
	double q[4];			// quaternion state
	double b[3];			// bias state
	double w[3];			// rate
	double q_dot[4];		// current q_dot
	double q_next[4];	// next q_dot

	// copy state variables
	memcpy(&q[0], &x_f64_a[0], sizeof(double)*4);
	memcpy(&b[0], &x_f64_a[4], sizeof(double)*3);

	// w = u - b
	sub64v(u, b, w, 3);

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
	normalise64(q_next, 4);
	memcpy(&x[0], &q_next[0], 4*sizeof(double));	// bias remains the same
}

void computeF(double F[7][7], double Ts, double x[7], double u[3]){
	int i, j;
	double w[3];
	double q[4];

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

	memcpy(&q[0], &x[0], 4*sizeof(double));

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

void computeW(double W[7][6], double Ts, double x[7]){
	int i, j;
	double q[4];

	memcpy(&q[0], &x[0], 4*sizeof(double));

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

void propagateCovariance(double P[7][7], double Ts, double gyro[3], double Q[6][6]){
	double F[7][7];
	double Ft[7][7];
	double W[7][6];
	double Wt[6][7];
	double FP[7][7] = {{0},{0}};
	double FPFt[7][7] = {{0},{0}};
	double WQ[7][6] = {{0},{0}};
	double WQWt[7][7] = {{0},{0}};

	computeF(F, Ts, x_f64_a, gyro);
	computeW(W, Ts, x_f64_a);

	transpose64(&F[0][0], &Ft[0][0], 7, 7);
	mul64m(&F[0][0], &P[0][0], &FP[0][0], 7, 7, 7);
	mul64m(&FP[0][0], &Ft[0][0], &FPFt[0][0], 7, 7, 7);

	transpose64(&W[0][0], &Wt[0][0], 7, 6);
	mul64m(&W[0][0], &Q[0][0], &WQ[0][0], 7, 6, 6);
	mul64m(&WQ[0][0], &Wt[0][0], &WQWt[0][0], 7, 6, 7);

	add64m(&FPFt[0][0], &WQWt[0][0], &P[0][0], 7, 7);
}

void updateStateAndCovariance(double x[7], double P[7][7], double R[6][6], double y[6]){
	double H[6][7] = {{0},{0}};
	double K[7][6] = {{0},{0}};
	double innovation[6] = {0};

	// compute H
	computeH(H, x, ywf_f64);
	// compute Kalman gain
	computeKalmanGain(K, H, P, R);
	// calculate innovation
	computeInnovation(x, y, ywf_f64, innovation);
	// correct state
	updateState(x, K, innovation);
	// update covariance
	updateCovariance(P, K, H, R);
}

void computeH(double H[6][7], double x[7], double y[6]){
	double q[4];
	copy64v(&q[0], &x[0], 4);

	double dR_dq0[3][3] = {{2.0 *  q[0], 2.0 *  q[3],2.0 * -q[2]},
							  {2.0 * -q[3], 2.0 *  q[0],2.0 *  q[1]},
							  {2.0 *  q[2], 2.0 * -q[1],2.0 *  q[0]}};

	double dR_dq1[3][3] = {{2.0 *  q[1], 2.0 *  q[2],2.0 *  q[3]},
							  {2.0 *  q[2], 2.0 * -q[1],2.0 *  q[0]},
							  {2.0 *  q[3], 2.0 * -q[0],2.0 * -q[1]}};

	double dR_dq2[3][3] = {{2.0 * -q[2], 2.0 *  q[1],2.0 * -q[0]},
							  {2.0 *  q[1], 2.0 *  q[2],2.0 *  q[3]},
							  {2.0 *  q[0], 2.0 *  q[3],2.0 * -q[2]}};

	double dR_dq3[3][3] = {{2.0 * -q[3], 2.0 *  q[0],2.0 *  q[1]},
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

int computeKalmanGain(double K[7][6], double H[6][7], double P[7][7], double R[6][6]){
	// K = P*H'/(H*P*H' + R)
	double Ht[7][6];
	double PHt[7][6];
	double HPHt[6][6];
	double HPHtpR[6][6];
	double K_den[6][6];
	double p[6][6];

	// transpose H
	transpose64(&H[0][0], &Ht[0][0], 6, 7);
	// matrix PHt
	mul64m(&P[0][0], &Ht[0][0], &PHt[0][0], 7, 7, 6);
	// matrix HPHt
	mul64m(&H[0][0], &PHt[0][0], &HPHt[0][0], 6, 7, 6);
	// K = PHt/(HPHt + R)
	add64m(&HPHt[0][0], &R[0][0], &HPHtpR[0][0], 6, 6);
	// K = PHt\(HPHt)
	if (cholsl(&HPHtpR[0][0], &K_den[0][0], &p[0][0], 6)) return 1;
	// K = PHt*K_den
	mul64m(&PHt[0][0], &K_den[0][0], &K[0][0], 7, 6, 6);
	return 0;
}

void computeInnovation(double x[7], double y[6], double ywf_f32[6], double inno[6]){
	double Rw2b[3][3];
	double q[4];
	copy64v(q, x, 4);
	q2R(Rw2b, q);

	double y_hat[6] = {0};
	int i,j;
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			y_hat[i] += Rw2b[i][j]*ywf_f32[j];
			y_hat[i+3] += Rw2b[i][j]*ywf_f32[j+3];
		}
	}

	sub64v(y, y_hat, inno, 6);
}

void updateState(double x[7], double K[7][6], double innovation[6]){
	double Ke[7] = {0};
	double x_cpy[7] = {0};
	copy64v(x_cpy, x, 7);

	int i,j;
	for (i = 0; i < 7; i++){
		for (j = 0; j < 6; j++){
			Ke[i] += K[i][j]*innovation[j];
		}
	}

	add64v(x_cpy, Ke, x, 7);
}

void updateCovariance(double P[7][7], double K[7][6], double H[6][7], double R[6][6]){
	double KH[7][7] = {{0},{0}};
	double I[7][7] = {{0},{0}};
	double IsKH[7][7] = {{0},{0}};
	double IsKHt[7][7] = {{0},{0}};
	double IsKHP[7][7] = {{0},{0}};
	double IsKHPIsKHt[7][7] = {{0},{0}};
	double Kt[6][7] = {{0},{0}};
	double KR[7][6] = {{0},{0}};
	double KRKt[7][7] = {{0},{0}};

	int i;
	for (i = 0; i < 7; i++)
		I[i][i] = 1;

	// P = (I - K*H)*P*(I - K*H)' + K*EKF.R*K';
	mul64m(&K[0][0], &H[0][0], &KH[0][0], 7, 6, 7);
	sub64m(&I[0][0], &KH[0][0], &IsKH[0][0], 7, 7);
	// P = IsKH*P*IsKH' + K*EKF.R*K';
	transpose64(&IsKH[0][0], &IsKHt[0][0], 7, 7);
	// P = IsKH*P*IsKHt + K*EKF.R*K';
	mul64m(&IsKH[0][0], &P[0][0], &IsKHP[0][0], 7, 7, 7);
	// P = IsKHP*IsKHt + K*EKF.R*K';
	mul64m(&IsKHP[0][0], &IsKHt[0][0], &IsKHPIsKHt[0][0], 7, 7, 7);
	// P = IsKHPIsKHt + K*EKF.R*K';
	transpose64(&K[0][0], &Kt[0][0], 7, 6);
	// P = IsKHPIsKHt + K*EKF.R*Kt;
	mul64m(&K[0][0], &R[0][0], &KR[0][0], 7, 6, 6);
	// P = IsKHPIsKHt + KR*Kt;
	mul64m(&KR[0][0], &Kt[0][0], &KRKt[0][0], 7, 6, 7);
	// P = IsKHPIsKHt + KRKt;
	add64m(&IsKHPIsKHt[0][0], &KRKt[0][0], &P[0][0], 7, 7);
}

void q2R(double R[3][3], double q[4]){
	R[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
	R[0][1] = 2.0*(q[1]*q[2] + q[0]*q[3]);
	R[0][2] = 2.0*(q[1]*q[3] - q[0]*q[2]);

	R[1][0] = 2.0*(q[1]*q[2] - q[0]*q[3]);
	R[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
	R[1][2] = 2.0*(q[2]*q[3] + q[0]*q[1]);

	R[2][0] = 2.0*(q[1]*q[3] + q[0]*q[2]);
	R[2][1] = 2.0*(q[2]*q[3] - q[0]*q[1]);
	R[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
}

void q2ypr(double q[4], double ypr[3]){
	ypr[0] = atan2(2.0*q[2]*q[3] + 2.0*q[0]*q[1], q[3]*q[3] - q[2]*q[2] - q[1]*q[1] + q[0]*q[0])*180.0/M_PI;
	ypr[1] = -asin(2.0*q[1]*q[3] - 2.0*q[0]*q[2])*180.0/M_PI;
	ypr[2] = atan2(2.0*q[1]*q[2] + 2.0*q[0]*q[3], q[1]*q[1] + q[0]*q[0] - q[3]*q[3] - q[2]*q[2])*180.0/M_PI;
}

#ifdef ARM
#include <arm_math.h>
void crossProduct3(float a[3], float b[3], float c[3]) {
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

void normalise32(float * v, int n){
	int i;
	float norm = 0;
	for (i = 0; i < n; i++){
		norm += v[i]*v[i];
	}
	norm = sqrt(norm);
	for (i = 0; i < n; i++){
		v[i] /= norm;
	}
}

void triadComputation(float r1_wf[3], float r2_wf[3], float r1_bf[3], float r2_bf[3], float ypr[3]){
	// find orthogonal vectors in both frame
	float r3_wf[3];
	float r13_wf[3];
	float r3_bf[3];
	float r13_bf[3];
	float Rw2b[3][3];
	float r1_wfc[3];
	float r2_wfc[3];

	for (int i=0; i < 3; i++){
		r1_wfc[i] = r1_wf[i];
		r2_wfc[i] = r2_wf[i];
	}

	normalise32(&r1_wf[0],3);
	normalise32(&r2_wf[0],3);
	normalise32(&r1_bf[0],3);
	normalise32(&r2_bf[0],3);

	crossProduct3(r1_wf, r2_wf, r3_wf);
	crossProduct3(r1_wf, r3_wf, r13_wf);	// orthogonal set r1 r13 r3
	crossProduct3(r1_bf, r2_bf, r3_bf);
	crossProduct3(r1_bf, r3_bf, r13_bf);
	// compute rotation matrix

	float bf[3][3];
	float wf[3][3];
	float wft[3][3];

	int i;
	for (i = 0; i < 3; i++){
		bf[i][0] = r1_bf[i];
		bf[i][1] = r13_bf[i];
		bf[i][2] = r3_bf[i];
		wf[i][0] = r1_wf[i];
		wf[i][1] = r13_wf[i];
		wf[i][2] = r3_wf[i];
	}

	arm_matrix_instance_f32 bf_m;
	arm_matrix_instance_f32 wf_m;
	arm_matrix_instance_f32 wft_m;
	arm_matrix_instance_f32 R_m;
	arm_mat_init_f32(&wf_m, 3, 3, &wf[0][0]);
	arm_mat_init_f32(&wft_m, 3, 3, &wft[0][0]);
	arm_mat_init_f32(&bf_m, 3, 3, &bf[0][0]);
	arm_mat_init_f32(&R_m, 3, 3, &Rw2b[0][0]);

	arm_mat_trans_f32(&wf_m, &wft_m);
	arm_mat_mult_f32(&bf_m, &wft_m, &R_m);

	ypr[0] = atan2(Rw2b[1][2], Rw2b[2][2])*180.0f/M_PI_f;
	ypr[1] = -asin(Rw2b[0][2])*180.0f/M_PI_f;
	ypr[2] = atan2(Rw2b[0][1], Rw2b[0][0])*180.0f/M_PI_f;
}
#endif
