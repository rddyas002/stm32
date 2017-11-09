#include "ekf.h"

float32_t P0_f32[] =
{
	0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1
};

float32_t x0_f32[] =
{
	1.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0
};

float32_t Q_f32[6][6];

float32_t x_f32[7];
float32_t P_f32[7][7];

void v32f_sub(float32_t y[3], const float32_t a[3], const float32_t b[3]);
void v32f_normalise4(float32_t v[4]);
void q2R(float32_t R[3][3], const float32_t q[4]);
void computeF(float32_t F[7][7], const float32_t Ts, const float32_t x[7], const float32_t u[3]);
void propagateState(float32_t x[7], const float32_t u[3], const float32_t Ts);
void computeW(float32_t W[7][6], const float32_t Ts, const float32_t x[7]);
void propagateCovariance(float32_t P[7][7], const float32_t F[7][7], const float32_t W[7][6], const float32_t Q[6][6]);

// initialise variables
void init_ekf(void){
	int i,j;
	for (i = 0; i < 7; i++){
		for (j = 0; j < 7; j++){
			P_f32[i][j] = 0.1;
		}
	}
	for (i = 0; i < 7; i++)
		x_f32[i] = x0_f32[i];
	for (i = 0; i < 3; i++)
		Q_f32[i][i] = 1.3539e-6f;
	for (i = 3; i < 6; i++)
		Q_f32[i][i] = 3.3846e-17f;

}

void run_ekf(float Ts, float gyro[3], float accel[3], float magnetic[3], float * q, float * w){
	float32_t R[3][3];
	float32_t F[7][7];
	float32_t W[7][6];
	float32_t q_tmp[4];

	// propagate state
	propagateState(x_f32, gyro, Ts);
	memcpy(&q_tmp[0], &x_f32[0], 4);
	q2R(R, q_tmp);		// use first 4 components

	// propagate covariance
	// compute F and W
	computeF(F, Ts, x_f32, gyro);
	computeW(W, Ts, x_f32);
	// propagate covariance matrix
	propagateCovariance(P_f32, F, W, Q_f32);
}

void propagateState(float32_t x[7], const float32_t u[3], const float32_t Ts){
	static float32_t q_dot_prev[2][4] = {{0, 0, 0, 0},{0, 0, 0, 0}};
	int i;
	float32_t q[4];			// quaternion state
	float32_t b[3];			// bias state
	float32_t w[3];			// rate
	float32_t q_dot[4];		// current q_dot
	float32_t q_next[4];	// next q_dot

	memcpy(&q[0], &x[0], 4*sizeof(float32_t));
	memcpy(&b[0], &x[4], 3*sizeof(float32_t));

	// w = u - b
	v32f_sub(w, u, b);

	// compute qdot
	q_dot[0] = 0.5*(-q[1]*w[0] + -q[2]*w[1] + -q[3]*w[2]);
	q_dot[1] = 0.5*( q[0]*w[0] + -q[3]*w[1] +  q[2]*w[2]);
	q_dot[2] = 0.5*( q[3]*w[0] +  q[0]*w[1] + -q[1]*w[2]);
	q_dot[3] = 0.5*(-q[2]*w[0] +  q[1]*w[1] +  q[0]*w[2]);

	// integrate to find q
	q_next[0] = q[0] + (Ts/6.0f)*(q_dot[0] + 4.0f*q_dot_prev[0][0] + q_dot_prev[1][0]);
	q_next[1] = q[1] + (Ts/6.0f)*(q_dot[1] + 4.0f*q_dot_prev[0][1] + q_dot_prev[1][1]);
	q_next[2] = q[2] + (Ts/6.0f)*(q_dot[2] + 4.0f*q_dot_prev[0][2] + q_dot_prev[1][2]);
	q_next[3] = q[3] + (Ts/6.0f)*(q_dot[3] + 4.0f*q_dot_prev[0][3] + q_dot_prev[1][3]);

	// shift q_dot and store
	for (i = 0; i < 4; i++){
		q_dot_prev[1][i] = q_dot_prev[0][i];
		q_dot_prev[0][i] = q_dot[i];
	}

	// normalise
	v32f_normalise4(q_next);
	memcpy(&x[0], &q_next[0], 4*sizeof(float32_t));	// bias remains the same
}

void computeF(float32_t F[7][7], const float32_t Ts, const float32_t x[7], const float32_t u[3]){
	int i, j;
	float32_t w[3];
	float32_t q[4];

	// initialise F
	for (i = 0; i < 7; i++){
		for (j = 0; j < 7; j++){
			F[i][j] = 0.0f;
		}
	}

	// calculate rate
	w[0] = u[0] - x[4];
	w[1] = u[1] - x[5];
	w[2] = u[2] - x[6];

	memcpy(&q[0], &x[0], 4*sizeof(float32_t));

	// F row 1
	F[0][0] = Ts*0.5f*0.0f;
	F[0][1] = Ts*0.5f*-w[0];
	F[0][2] = Ts*0.5f*-w[1];
	F[0][3] = Ts*0.5f*-w[2];
	F[0][4] = Ts*0.5f*q[1];
	F[0][5] = Ts*0.5f*q[2];
	F[0][6] = Ts*0.5f*q[3];

	// F row 2
	F[1][0] = Ts*0.5f*w[0];
	F[1][1] = Ts*0.5f*0.0f;
	F[1][2] = Ts*0.5f*w[2];
	F[1][3] = Ts*0.5f*-w[1];
	F[1][4] = Ts*0.5f*-q[0];
	F[1][5] = Ts*0.5f*q[3];
	F[1][6] = Ts*0.5f*-q[2];

	// F row 3
	F[2][0] = Ts*0.5f*w[1];
	F[2][1] = Ts*0.5f*-w[2];
	F[2][2] = Ts*0.5f*0.0f;
	F[2][3] = Ts*0.5f*w[0];
	F[2][4] = Ts*0.5f*-q[3];
	F[2][5] = Ts*0.5f*-q[0];
	F[2][6] = Ts*0.5f*q[1];

	// F row 4
	F[3][0] = Ts*0.5f*w[2];
	F[3][1] = Ts*0.5f*w[1];
	F[3][2] = Ts*0.5f*-w[0];
	F[3][3] = Ts*0.5f*0.0f;
	F[3][4] = Ts*0.5f*q[2];
	F[3][5] = Ts*0.5f*-q[1];
	F[3][6] = Ts*0.5f*-q[0];

	// F row 4
	F[4][0] = Ts*0.5f*w[1];
	F[4][1] = Ts*0.5f*-w[2];
	F[4][2] = Ts*0.5f*0.0f;
	F[4][3] = Ts*0.5f*w[0];
	F[4][4] = Ts*0.5f*-q[3];
	F[4][5] = Ts*0.5f*-q[0];
	F[4][6] = Ts*0.5f*q[1];

	// rest of the rows are zero

	// add identity to F
	for (i = 0; i < 7; i++)
		F[i][i] += 1;
}

void computeW(float32_t W[7][6], const float32_t Ts, const float32_t x[7]){
	int i, j;
	float32_t q[4];

	memcpy(&q[0], &x[0], 4*sizeof(float32_t));

	// initialise W
	for (i = 0; i < 7; i++){
		for (j = 0; j < 6; j++){
			W[i][j] = 0.0f;
		}
	}

	W[0][0] = Ts*0.5f*-q[1];
	W[0][1] = Ts*0.5f*-q[2];
	W[0][2] = Ts*0.5f*-q[3];
	// W[0][3] ... W[0][5] = 0;

	W[1][0] = Ts*0.5f*q[0];
	W[1][1] = Ts*0.5f*-q[3];
	W[1][2] = Ts*0.5f*q[2];
	// W[1][3] ... W[1][5] = 0;

	W[2][0] = Ts*0.5f*q[3];
	W[2][1] = Ts*0.5f*q[0];
	W[2][2] = Ts*0.5f*-q[1];
	// W[2][3] ... W[2][5] = 0;

	W[3][0] = Ts*0.5f*-q[2];
	W[3][1] = Ts*0.5f*q[1];
	W[3][2] = Ts*0.5f*q[0];
	// W[3][3] ... W[3][5] = 0;

	// W[4][0] ... W[4][2] = 0;
	W[4][3] = 1.0f;
	// W[4][4] ... W[3][5] = 0;

	// W[5][0] ... W[5][3] = 0;
	W[5][4] = 1.0f;
	// W[5][4] ... W[5][5] = 0;

	// W[6][0] ... W[6][4] = 0;
	W[6][5] = 1.0f;
}

void propagateCovariance(float32_t P[7][7], const float32_t F[7][7], const float32_t W[7][6], const float32_t Q[6][6]){
	int i,j,k;
	float32_t FP[7][6] = {{0},{0}};
	float32_t FPFt[7][7] = {{0},{0}};
	float32_t WQ[7][6] = {{0},{0}};
	float32_t WQWt[7][7] = {{0},{0}};
	float32_t sum_fp, sum_wq;

	// P = F*P*F' + W*Q*W'

	// FP = F*P
	// WQ = W*Q
	for (i = 0; i < 7; i++){
		for (j = 0; j < 6; j++){
			sum_fp = 0.0f;
			sum_wq = 0.0f;
			for (k = 0; k < 7; k++){
				sum_fp += F[i][k]*P[k][j];
				sum_wq += W[i][k]*Q[k][j];
			}
			FP[i][j] = sum_fp;
			WQ[i][j] = sum_wq;
		}
	}
	// P = FP*F' + WQ*W'

	for (i = 0; i < 7; i++){
		for (j = 0; j < 7; j++){
			sum_fp = 0.0f;
			sum_wq = 0.0f;
			for (k = 0; k < 6; k++){
				sum_fp += FP[i][k]*F[j][k];
				sum_wq += WQ[i][k]*W[j][k];
			}
			FPFt[i][j] = sum_fp;
			WQWt[i][j] = sum_wq;
		}
	}

	// P = FPFt + WQWt
	for (i = 0; i < 7; i++)
		for (j = 0; j < 7; j++)
			P[i][j] = FPFt[i][j] + WQWt[i][j];
}

void q2R(float32_t R[3][3], const float32_t q[4]){
	R[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
	R[0][1] = 2.0f*(q[1]*q[2] + q[0]*q[3]);
	R[0][2] = 2.0f*(q[1]*q[3] - q[0]*q[2]);

	R[1][0] = 2.0f*(q[1]*q[2] - q[0]*q[3]);
	R[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
	R[1][2] = 2.0f*(q[2]*q[3] + q[0]*q[1]);

	R[2][0] = 2.0f*(q[1]*q[3] + q[0]*q[2]);
	R[2][1] = 2.0f*(q[2]*q[3] - q[0]*q[1]);
	R[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
}

void v32f_sub(float32_t y[3], const float32_t a[3], const float32_t b[3]){
	y[0] = a[0] - b[0];
	y[1] = a[1] - b[1];
	y[2] = a[2] - b[2];
}

void v32f_normalise4(float32_t v[4]){
	float32_t norm = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
	v[0] /= norm;
	v[1] /= norm;
	v[2] /= norm;
	v[3] /= norm;
}
