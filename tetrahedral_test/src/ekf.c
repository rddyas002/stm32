#include "ekf.h"

// State error covariance matrix
const float32_t P0_f32_a[7][7] =
{
	{0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
	{0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0},
	{0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0},
	{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1}
};

// State vector
const float32_t x0_f32_a[7] =
{
	1.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0,
	0.0
};

float32_t Q_f32_a[6][6];		// Process noise covariance
float32_t R_f32_a[6][6];		// Measurement noise covariance
float32_t x_f32_a[7];			// State vector
float32_t P_f32_a[7][7];		// Error covariance
float32_t ywf_f32[6];			// World frame origin measurement reference

void v32f_sub(float32_t y[3], const float32_t a[3], const float32_t b[3]);
void v32f_normalise4(float32_t v[4]);
void q2R(float32_t R[3][3], const float32_t q[4]);
void computeF(float32_t F[7][7], const float32_t Ts, const float32_t x[7], const float32_t u[3]);
void propagateState(float32_t x[7], const float32_t u[3], const float32_t Ts);
void computeW(float32_t W[7][6], const float32_t Ts, const float32_t x[7]);
void propagateCovariance(float32_t P[7][7], const float32_t Ts, const float32_t gyro[3], const float32_t Q[6][6]);
void updateStateAndCovariance(float32_t x[7], float32_t P[7][7], const float32_t R[6][6], const float32_t y[6]);
void computeH(float32_t H[6][7], const float32_t x[7], const float32_t y[6]);
void computeKalmanGain(float32_t K[7][6], const float32_t H[6][7], const float32_t P[7][7], const float32_t R[6][6]);
void computeInnovation(float32_t x[7], float32_t y[6], float32_t ywf_f32[6], float32_t inno[6]);
void updateState(float32_t x[7], float32_t K[7][6], float32_t innovation[6]);
void updateCovariance(float32_t P[7][7], float32_t K[7][6], float32_t H[6][7]);

// initialise variables
void init_ekf(float32_t ywf[6]){
	int i;
	for (i = 0; i < 7; i++)
		P_f32_a[i][i] = 0.1;
	for (i = 0; i < 7; i++)
		x_f32_a[i] = x0_f32_a[i];
	for (i = 0; i < 3; i++)
		Q_f32_a[i][i] = 1.3539e-6f;
	for (i = 3; i < 6; i++)
		Q_f32_a[i][i] = 3.3846e-17f;
	for (i = 0; i < 3; i++)
		R_f32_a[i][i] = 7.6147e-05f;
	for (i = 3; i < 6; i++)
		R_f32_a[i][i] = 8.4521e-04f;

	for (i = 0; i < 6; i++)
		ywf_f32[i] = ywf[i];
}

void run_ekf(float Ts, float gyro[3], float accel[3], float magnetic[3], float * q, float * w){
	float32_t y[6];

	float accel_norm = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
	float mag_norm = sqrt(magnetic[0]*magnetic[0] + magnetic[1]*magnetic[1] + magnetic[2]*magnetic[2]);
	y[0] = accel[0]/accel_norm;
	y[1] = accel[1]/accel_norm;
	y[2] = accel[2]/accel_norm;
	y[3] = magnetic[0]/mag_norm;
	y[4] = magnetic[1]/mag_norm;
	y[5] = magnetic[2]/mag_norm;

	// propagate state
	propagateState(x_f32_a, gyro, Ts);
	// propagate covariance
	propagateCovariance(P_f32_a, Ts, gyro, Q_f32_a);
	// update phase
	updateStateAndCovariance(x_f32_a, P_f32_a, R_f32_a, y);

	arm_copy_f32(&x_f32_a[0], q, 4);
	arm_copy_f32(&x_f32_a[3], w, 3);
}

void propagateState(float32_t x[7], const float32_t u[3], const float32_t Ts){
	static float32_t q_dot_prev[2][4] = {{0, 0, 0, 0},{0, 0, 0, 0}};
	int i;
	float32_t q[4];			// quaternion state
	float32_t b[3];			// bias state
	float32_t w[3];			// rate
	float32_t q_dot[4];		// current q_dot
	float32_t q_next[4];	// next q_dot

	arm_copy_f32(&x[0], &q[0], 4);
	arm_copy_f32(&x[4], &b[0], 3);

	// w = u - b
	arm_sub_f32(&u[0], &b[0], &w[0], 3);

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
	//float32_t q_norm;
	//float32_t q_normalised[4];
	//arm_dot_prod_f32(&q_next[0], &q_next[0], 4, &q_norm);
	//arm_scale_f32(&q_next[0], 1/q_norm, &x[0], 4);

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

void propagateCovariance(float32_t P[7][7], const float32_t Ts, const float32_t gyro[3], const float32_t Q[6][6]){
	int i,j,k;
	float32_t F[7][7];
	float32_t W[7][6];
	float32_t FP[7][7] = {{0},{0}};
	float32_t FPFt[7][7] = {{0},{0}};
	float32_t WQ[7][6] = {{0},{0}};
	float32_t WQWt[7][7] = {{0},{0}};
	float32_t sum_fp, sum_wq;

	computeF(F, Ts, x_f32_a, gyro);
	computeW(W, Ts, x_f32_a);

	// P = F*P*F' + W*Q*W'
	//arm_mat_mult_f32(&F_f32_m, &P_f32_m, &FP_f32_m);

	// FP = F*P
	// WQ = W*Q
	for (i = 0; i < 7; i++){
		for (j = 0; j < 7; j++){
			sum_fp = 0.0f;
			for (k = 0; k < 7; k++){
				sum_fp += F[i][k]*P[k][j];
			}
			FP[i][j] = sum_fp;
		}
	}

	for (i = 0; i < 7; i++){
		for (j = 0; j < 6; j++){
			sum_wq = 0.0f;
			for (k = 0; k < 6; k++){
				sum_wq += W[i][k]*Q[k][j];
			}
			WQ[i][j] = sum_wq;
		}
	}
	// P = FP*F' + WQ*W'

	//FPF'
	for (i = 0; i < 7; i++){
		for (j = 0; j < 7; j++){
			sum_fp = 0.0f;
			for (k = 0; k < 7; k++){
				sum_fp += FP[i][k]*F[j][k];
			}
			FPFt[i][j] = sum_fp;
		}
	}

	//WQW'
	for (i = 0; i < 7; i++){
		for (j = 0; j < 7; j++){
			sum_wq = 0.0f;
			for (k = 0; k < 6; k++){
				sum_wq += WQ[i][k]*W[j][k];
			}
			WQWt[i][j] = sum_wq;
		}
	}

	// P = FPFt + WQWt
	for (i = 0; i < 7; i++)
		for (j = 0; j < 7; j++)
			P[i][j] = FPFt[i][j] + WQWt[i][j];
}

void updateStateAndCovariance(float32_t x[7], float32_t P[7][7], const float32_t R[6][6], const float32_t y[6]){
	float32_t H[6][7] = {{0},{0}};
	float32_t K[7][6] = {{0},{0}};
	float32_t innovation[6];

	// compute H
	computeH(H, x, ywf_f32);
	// compute Kalman gain
	computeKalmanGain(K, H, P, R);
	// calculate innovation
	computeInnovation(x, y, ywf_f32, innovation);
	// correct state
	updateState(x, K, innovation);
	// update covariance
	updateCovariance(P, K, H);
}

void computeH(float32_t H[6][7], const float32_t x[7], const float32_t y[6]){
	float32_t q[4];
	arm_copy_f32(&x[0], &q[0], 4);

	float32_t dR_dq0[3][3] = {{2.0f *  q[0], 2.0f *  q[3],2.0f * -q[2]},
							  {2.0f * -q[3], 2.0f *  q[0],2.0f *  q[1]},
							  {2.0f *  q[2], 2.0f * -q[1],2.0f *  q[0]}};

	float32_t dR_dq1[3][3] = {{2.0f *  q[1], 2.0f *  q[2],2.0f *  q[3]},
							  {2.0f *  q[2], 2.0f * -q[1],2.0f *  q[0]},
							  {2.0f *  q[3], 2.0f * -q[0],2.0f * -q[1]}};

	float32_t dR_dq2[3][3] = {{2.0f * -q[2], 2.0f *  q[1],2.0f * -q[0]},
							  {2.0f *  q[1], 2.0f *  q[2],2.0f *  q[3]},
							  {2.0f *  q[0], 2.0f *  q[3],2.0f * -q[2]}};

	float32_t dR_dq3[3][3] = {{2.0f * -q[3], 2.0f *  q[0],2.0f *  q[1]},
							  {2.0f * -q[0], 2.0f * -q[3],2.0f *  q[2]},
							  {2.0f *  q[1], 2.0f *  q[2],2.0f *  q[3]}};

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

void computeKalmanGain(float32_t K[7][6], const float32_t H[6][7], const float32_t P[7][7], const float32_t R[6][6]){
	// K = P*H'/(H*P*H' + R)

	float32_t Ht[7][6];
	float32_t PHt[7][6];
	float32_t HPHt[6][6];
	float32_t K_den[6][6];

	arm_matrix_instance_f32 P_f32_m;
	arm_matrix_instance_f32 H_f32_m;
	arm_matrix_instance_f32 Ht_f32_m;
	arm_matrix_instance_f32 PHt_f32_m;
	arm_matrix_instance_f32 HPHt_f32_m;
	arm_matrix_instance_f32 R_f32_m;
	arm_matrix_instance_f32 K_den_f32_m;
	arm_matrix_instance_f32 K_f32_m;

	arm_mat_init_f32(&P_f32_m, 7, 7, &P[0][0]);
	arm_mat_init_f32(&H_f32_m, 6, 7, &H[0][0]);
	arm_mat_init_f32(&Ht_f32_m, 7, 6, &Ht[0][0]);
	arm_mat_init_f32(&PHt_f32_m, 7, 6, &PHt[0][0]);
	arm_mat_init_f32(&HPHt_f32_m, 6, 6, &HPHt[0][0]);
	arm_mat_init_f32(&R_f32_m, 6, 6, &R[0][0]);
	arm_mat_init_f32(&K_den_f32_m, 6, 6, &K_den[0][0]);
	arm_mat_init_f32(&K_f32_m, 7, 6, &K[0][0]);

	// transpose H
	arm_mat_trans_f32(&H_f32_m, &Ht_f32_m);
	// matrix PHt
	arm_mat_mult_f32(&P_f32_m, &Ht_f32_m, &PHt_f32_m);
	// matrix HPHt
	arm_mat_mult_f32(&H_f32_m, &PHt_f32_m, &HPHt_f32_m);
	// K = PHt/(HPHt + R)
	arm_mat_add_f32(&HPHt_f32_m, &R_f32_m, &HPHt_f32_m);
	// K = PHt/(HPHt)
	arm_mat_inverse_f32(&HPHt_f32_m, &K_den_f32_m);
	// K = PHt*K_den
	arm_mat_mult_f32(&PHt_f32_m, &K_den_f32_m, &K_f32_m);
}

void computeInnovation(float32_t x[7], float32_t y[6], float32_t ywf_f32[6], float32_t inno[6]){
	float32_t Rw2b[3][3];
	float32_t q[4];
	arm_copy_f32(&x[0], &q[0], 4);
	q2R(Rw2b, q);

	float32_t y_hat[6] = {0};
	int i,j;
	for (i = 0; i < 3; i++){
		for (j = 0; j < 3; j++){
			y_hat[i] += Rw2b[i][j]*ywf_f32[j];
			y_hat[i+3] += Rw2b[i][j]*ywf_f32[j+3];
		}
	}

	arm_sub_f32(&y[0], &y_hat[0], &inno[0], 6);
}

void updateState(float32_t x[7], float32_t K[7][6], float32_t innovation[6]){
	float32_t Ke[7] = {0};
	float32_t x_cpy[7] = {0};
	arm_copy_f32(&x[0], &x_cpy[0], 7);

	int i,j;
	for (i = 0; i < 7; i++){
		for (j = 0; j < 6; j++){
			Ke[i] += K[i][j]*innovation[j];
		}
	}

	arm_add_f32(&x_cpy[0], &Ke[0], &x[0], 7);
}

void updateCovariance(float32_t P[7][7], float32_t K[7][6], float32_t H[6][7]){
	float32_t KH[7][7];
	float32_t P2[7][7];
	float32_t eye7[7][7] = {{0},{0}};

	int i;
	for (i = 0; i < 7; i++)
		eye7[i][i] = 1.0f;

	arm_matrix_instance_f32 P_f32_m;
	arm_matrix_instance_f32 P2_f32_m;
	arm_matrix_instance_f32 H_f32_m;
	arm_matrix_instance_f32 K_f32_m;
	arm_matrix_instance_f32 KH_f32_m;
	arm_matrix_instance_f32 eye7_f32_m;

	arm_mat_init_f32(&P_f32_m, 7, 7, &P[0][0]);
	arm_mat_init_f32(&P2_f32_m, 7, 7, &P2[0][0]);
	arm_mat_init_f32(&K_f32_m, 7, 6, &K[0][0]);
	arm_mat_init_f32(&H_f32_m, 6, 7, &H[0][0]);
	arm_mat_init_f32(&KH_f32_m, 7, 7, &KH[0][0]);
	arm_mat_init_f32(&eye7_f32_m, 7, 7, &eye7[0][0]);

	arm_mat_mult_f32(&K_f32_m, &H_f32_m, &KH_f32_m);
	arm_mat_sub_f32(&eye7_f32_m, &KH_f32_m, &KH_f32_m);
	arm_mat_mult_f32(&KH_f32_m, &P_f32_m, &P2_f32_m);

	memcpy(&P[0][0], &P2[0][0], sizeof(float32_t)*7*7);
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

void q2ypr(float32_t q[4], float32_t ypr[3]){
	ypr[0] = atan2(2.0f*q[2]*q[3] + 2.0f*q[0]*q[1], q[3]*q[3] - q[2]*q[2] - q[1]*q[1] + q[0]*q[0])*180.0f/M_PI_f;
	ypr[1] = -asin(2.0f*q[1]*q[3] - 2.0f*q[0]*q[2])*180.0f/M_PI_f;
	ypr[2] = atan2(2.0f*q[1]*q[2] + 2.0f*q[0]*q[3], q[1]*q[1] + q[0]*q[0] - q[3]*q[3] - q[2]*q[2])*180.0f/M_PI_f;
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
