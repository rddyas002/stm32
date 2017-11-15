#include "ekf.h"

// State error covariance matrix
const float32_t P0_f32_a[7][7] =
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

typedef struct
{
  uint16_t numRows;     /**< number of rows of the matrix.     */
  uint16_t numCols;     /**< number of columns of the matrix.  */
  float64_t *pData;     /**< points to the data of the matrix. */
} arm_matrix_instance_f64;
arm_status arm_mat_inverse_f64(const arm_matrix_instance_f64 * pSrc, arm_matrix_instance_f64 * pDst);

void arm_mat_init_f64(
  arm_matrix_instance_f64 * S,
  uint16_t nRows,
  uint16_t nColumns,
  float64_t * pData)
{
  /* Assign Number of Rows */
  S->numRows = nRows;

  /* Assign Number of Columns */
  S->numCols = nColumns;

  /* Assign Data pointer */
  S->pData = pData;
}

float32_t Q_f32_a[6][6] = {0};		// Process noise covariance
float32_t R_f32_a[6][6] = {0};		// Measurement noise covariance
float32_t x_f32_a[7] = {0};			// State vector
float32_t P_f32_a[7][7] = {0};		// Error covariance
float32_t ywf_f32[6] = {0};			// World frame origin measurement reference

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
void crossProduct3(float32_t a[3], float32_t b[3], float32_t c[3]);

// https://github.com/simondlevy/TinyEKF/blob/master/tiny_ekf.c

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
		Q_f32_a[i][i] = 0.35e-5f;
	for (i = 3; i < 6; i++)
		Q_f32_a[i][i] = 2.5e-8f;

	// accel measurement noise
	for (i = 0; i < 3; i++)
		R_f32_a[i][i] = 20.0f;//0.3e-4f;
	// magnetometer noise
	for (i = 3; i < 6; i++)
		R_f32_a[i][i] = 20.0f;//0.4e-4f;

	for (i = 0; i < 3; i++){
		ywf_f32[i] = imu_data->accel_offset[i];
		ywf_f32[i+3] = imu_data->mag_offset[i];
		x_f32_a[i+4] = imu_data->gyro_offset[i];
	}
}

void run_ekf(float32_t Ts, float32_t gyro[3], float32_t accel[3], float32_t magnetic[3], float32_t * q, float32_t * w){
	float32_t y[6];

	y[0] = accel[0];
	y[1] = accel[1];
	y[2] = accel[2];
	y[3] = magnetic[0];
	y[4] = magnetic[1];
	y[5] = magnetic[2];

	// propagate state
	propagateState(x_f32_a, gyro, Ts);
	// propagate covariance
	propagateCovariance(P_f32_a, Ts, gyro, Q_f32_a);
	// update phase
	//updateStateAndCovariance(x_f32_a, P_f32_a, R_f32_a, y);

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
	float32_t F[7][7];
	float32_t Ft[7][7];
	float32_t W[7][6];
	float32_t Wt[6][7];
	float32_t FP[7][7] = {{0},{0}};
	float32_t FPFt[7][7] = {{0},{0}};
	float32_t WQ[7][6] = {{0},{0}};
	float32_t WQWt[7][7] = {{0},{0}};

	arm_matrix_instance_f32 F_f32_m;
	arm_matrix_instance_f32 P_f32_m;
	arm_matrix_instance_f32 Ft_f32_m;
	arm_matrix_instance_f32 FP_f32_m;
	arm_matrix_instance_f32 FPFt_f32_m;
	arm_matrix_instance_f32 W_f32_m;
	arm_matrix_instance_f32 Q_f32_m;
	arm_matrix_instance_f32 Wt_f32_m;
	arm_matrix_instance_f32 WQ_f32_m;
	arm_matrix_instance_f32 WQWt_f32_m;

	arm_mat_init_f32(&F_f32_m, 7, 7, &F[0][0]);
	arm_mat_init_f32(&Ft_f32_m, 7, 7, &Ft[0][0]);
	arm_mat_init_f32(&FP_f32_m, 7, 7, &FP[0][0]);
	arm_mat_init_f32(&FPFt_f32_m, 7, 7, &FPFt[0][0]);
	arm_mat_init_f32(&P_f32_m, 7, 7, &P[0][0]);
	arm_mat_init_f32(&W_f32_m, 7, 6, &W[0][0]);
	arm_mat_init_f32(&Q_f32_m, 6, 6, &Q[0][0]);
	arm_mat_init_f32(&Wt_f32_m, 6, 7, &Wt[0][0]);
	arm_mat_init_f32(&WQ_f32_m, 7, 6, &WQ[0][0]);
	arm_mat_init_f32(&WQWt_f32_m, 7, 7, &WQWt[0][0]);

	computeF(F, Ts, x_f32_a, gyro);	// checked
	computeW(W, Ts, x_f32_a);		// checked

	arm_mat_trans_f32(&F_f32_m, &Ft_f32_m);
	arm_mat_mult_f32(&F_f32_m, &P_f32_m, &FP_f32_m);
	arm_mat_mult_f32(&FP_f32_m, &Ft_f32_m, &FPFt_f32_m);

	arm_mat_trans_f32(&W_f32_m, &Wt_f32_m);
	arm_mat_mult_f32(&W_f32_m, &Q_f32_m, &WQ_f32_m);
	arm_mat_mult_f32(&WQ_f32_m, &Wt_f32_m, &WQWt_f32_m);

	arm_mat_add_f32(&FPFt_f32_m, &WQWt_f32_m, &P_f32_m);
}

void updateStateAndCovariance(float32_t x[7], float32_t P[7][7], const float32_t R[6][6], const float32_t y[6]){
	float32_t H[6][7] = {{0},{0}};
	float32_t K[7][6] = {{0},{0}};
	float32_t innovation[6] = {0};

	// compute H
	computeH(H, x, ywf_f32);
	//everything above verified high accuracy

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
	float32_t HPHtpR[6][6];
	float32_t K_den[6][6];

	arm_matrix_instance_f32 P_f32_m;
	arm_matrix_instance_f32 H_f32_m;
	arm_matrix_instance_f32 Ht_f32_m;
	arm_matrix_instance_f32 PHt_f32_m;
	arm_matrix_instance_f32 HPHt_f32_m;
	arm_matrix_instance_f32 HPHtpR_f32_m;
	arm_matrix_instance_f32 R_f32_m;
	arm_matrix_instance_f32 K_den_f32_m;
	arm_matrix_instance_f32 K_f32_m;

	arm_mat_init_f32(&P_f32_m, 7, 7, &P[0][0]);
	arm_mat_init_f32(&H_f32_m, 6, 7, &H[0][0]);
	arm_mat_init_f32(&Ht_f32_m, 7, 6, &Ht[0][0]);
	arm_mat_init_f32(&PHt_f32_m, 7, 6, &PHt[0][0]);
	arm_mat_init_f32(&HPHt_f32_m, 6, 6, &HPHt[0][0]);
	arm_mat_init_f32(&HPHtpR_f32_m, 6, 6, &HPHtpR[0][0]);
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
	arm_mat_add_f32(&HPHt_f32_m, &R_f32_m, &HPHtpR_f32_m);
	// K = PHt\(HPHt)
	arm_status inv_status = arm_mat_inverse_f32(&HPHtpR_f32_m, &K_den_f32_m);
	// K = PHt*K_den
	arm_mat_mult_f32(&PHt_f32_m, &K_den_f32_m, &K_f32_m);
}


arm_status arm_mat_inverse_f64(
  const arm_matrix_instance_f64 * pSrc,
  arm_matrix_instance_f64 * pDst)
{
  float64_t *pIn = pSrc->pData;                  /* input data matrix pointer */
  float64_t *pOut = pDst->pData;                 /* output data matrix pointer */
  float64_t *pInT1, *pInT2;                      /* Temporary input data matrix pointer */
  float64_t *pOutT1, *pOutT2;                    /* Temporary output data matrix pointer */
  float64_t *pPivotRowIn, *pPRT_in, *pPivotRowDst, *pPRT_pDst;  /* Temporary input and output data matrix pointer */
  uint32_t numRows = pSrc->numRows;              /* Number of rows in the matrix  */
  uint32_t numCols = pSrc->numCols;              /* Number of Cols in the matrix  */

#ifndef ARM_MATH_CM0_FAMILY
  float64_t maxC;                                /* maximum value in the column */

  /* Run the below code for Cortex-M4 and Cortex-M3 */

  float64_t Xchg, in = 0.0f, in1;                /* Temporary input values  */
  uint32_t i, rowCnt, flag = 0u, j, loopCnt, k, l;      /* loop counters */
  arm_status status;                             /* status of matrix inverse */

#ifdef ARM_MATH_MATRIX_CHECK


  /* Check for matrix mismatch condition */
  if((pSrc->numRows != pSrc->numCols) || (pDst->numRows != pDst->numCols)
     || (pSrc->numRows != pDst->numRows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*    #ifdef ARM_MATH_MATRIX_CHECK    */

  {

    /*--------------------------------------------------------------------------------------------------------------
	 * Matrix Inverse can be solved using elementary row operations.
	 *
	 *	Gauss-Jordan Method:
	 *
	 *	   1. First combine the identity matrix and the input matrix separated by a bar to form an
	 *        augmented matrix as follows:
	 *				        _ 	      	       _         _	       _
	 *					   |  a11  a12 | 1   0  |       |  X11 X12  |
	 *					   |           |        |   =   |           |
	 *					   |_ a21  a22 | 0   1 _|       |_ X21 X21 _|
	 *
	 *		2. In our implementation, pDst Matrix is used as identity matrix.
	 *
	 *		3. Begin with the first row. Let i = 1.
	 *
	 *	    4. Check to see if the pivot for column i is the greatest of the column.
	 *		   The pivot is the element of the main diagonal that is on the current row.
	 *		   For instance, if working with row i, then the pivot element is aii.
	 *		   If the pivot is not the most significant of the columns, exchange that row with a row
	 *		   below it that does contain the most significant value in column i. If the most
	 *         significant value of the column is zero, then an inverse to that matrix does not exist.
	 *		   The most significant value of the column is the absolute maximum.
	 *
	 *	    5. Divide every element of row i by the pivot.
	 *
	 *	    6. For every row below and  row i, replace that row with the sum of that row and
	 *		   a multiple of row i so that each new element in column i below row i is zero.
	 *
	 *	    7. Move to the next row and column and repeat steps 2 through 5 until you have zeros
	 *		   for every element below and above the main diagonal.
	 *
	 *		8. Now an identical matrix is formed to the left of the bar(input matrix, pSrc).
	 *		   Therefore, the matrix to the right of the bar is our solution(pDst matrix, pDst).
	 *----------------------------------------------------------------------------------------------------------------*/

    /* Working pointer for destination matrix */
    pOutT1 = pOut;

    /* Loop over the number of rows */
    rowCnt = numRows;

    /* Making the destination matrix as identity matrix */
    while(rowCnt > 0u)
    {
      /* Writing all zeroes in lower triangle of the destination matrix */
      j = numRows - rowCnt;
      while(j > 0u)
      {
        *pOutT1++ = 0.0f;
        j--;
      }

      /* Writing all ones in the diagonal of the destination matrix */
      *pOutT1++ = 1.0f;

      /* Writing all zeroes in upper triangle of the destination matrix */
      j = rowCnt - 1u;
      while(j > 0u)
      {
        *pOutT1++ = 0.0f;
        j--;
      }

      /* Decrement the loop counter */
      rowCnt--;
    }

    /* Loop over the number of columns of the input matrix.
       All the elements in each column are processed by the row operations */
    loopCnt = numCols;

    /* Index modifier to navigate through the columns */
    l = 0u;

    while(loopCnt > 0u)
    {
      /* Check if the pivot element is zero..
       * If it is zero then interchange the row with non zero row below.
       * If there is no non zero element to replace in the rows below,
       * then the matrix is Singular. */

      /* Working pointer for the input matrix that points
       * to the pivot element of the particular row  */
      pInT1 = pIn + (l * numCols);

      /* Working pointer for the destination matrix that points
       * to the pivot element of the particular row  */
      pOutT1 = pOut + (l * numCols);

      /* Temporary variable to hold the pivot value */
      in = *pInT1;

      /* Grab the most significant value from column l */
      maxC = 0;
      for (i = l; i < numRows; i++)
      {
        maxC = *pInT1 > 0 ? (*pInT1 > maxC ? *pInT1 : maxC) : (-*pInT1 > maxC ? -*pInT1 : maxC);
        pInT1 += numCols;
      }

      /* Update the status if the matrix is singular */
      if(maxC == 0.0f)
      {
        return ARM_MATH_SINGULAR;
      }

      /* Restore pInT1  */
      pInT1 = pIn;

      /* Destination pointer modifier */
      k = 1u;

      /* Check if the pivot element is the most significant of the column */
      if( (in > 0.0f ? in : -in) != maxC)
      {
        /* Loop over the number rows present below */
        i = numRows - (l + 1u);

        while(i > 0u)
        {
          /* Update the input and destination pointers */
          pInT2 = pInT1 + (numCols * l);
          pOutT2 = pOutT1 + (numCols * k);

          /* Look for the most significant element to
           * replace in the rows below */
          if((*pInT2 > 0.0f ? *pInT2: -*pInT2) == maxC)
          {
            /* Loop over number of columns
             * to the right of the pilot element */
            j = numCols - l;

            while(j > 0u)
            {
              /* Exchange the row elements of the input matrix */
              Xchg = *pInT2;
              *pInT2++ = *pInT1;
              *pInT1++ = Xchg;

              /* Decrement the loop counter */
              j--;
            }

            /* Loop over number of columns of the destination matrix */
            j = numCols;

            while(j > 0u)
            {
              /* Exchange the row elements of the destination matrix */
              Xchg = *pOutT2;
              *pOutT2++ = *pOutT1;
              *pOutT1++ = Xchg;

              /* Decrement the loop counter */
              j--;
            }

            /* Flag to indicate whether exchange is done or not */
            flag = 1u;

            /* Break after exchange is done */
            break;
          }

          /* Update the destination pointer modifier */
          k++;

          /* Decrement the loop counter */
          i--;
        }
      }

      /* Update the status if the matrix is singular */
      if((flag != 1u) && (in == 0.0f))
      {
        return ARM_MATH_SINGULAR;
      }

      /* Points to the pivot row of input and destination matrices */
      pPivotRowIn = pIn + (l * numCols);
      pPivotRowDst = pOut + (l * numCols);

      /* Temporary pointers to the pivot row pointers */
      pInT1 = pPivotRowIn;
      pInT2 = pPivotRowDst;

      /* Pivot element of the row */
      in = *pPivotRowIn;

      /* Loop over number of columns
       * to the right of the pilot element */
      j = (numCols - l);

      while(j > 0u)
      {
        /* Divide each element of the row of the input matrix
         * by the pivot element */
        in1 = *pInT1;
        *pInT1++ = in1 / in;

        /* Decrement the loop counter */
        j--;
      }

      /* Loop over number of columns of the destination matrix */
      j = numCols;

      while(j > 0u)
      {
        /* Divide each element of the row of the destination matrix
         * by the pivot element */
        in1 = *pInT2;
        *pInT2++ = in1 / in;

        /* Decrement the loop counter */
        j--;
      }

      /* Replace the rows with the sum of that row and a multiple of row i
       * so that each new element in column i above row i is zero.*/

      /* Temporary pointers for input and destination matrices */
      pInT1 = pIn;
      pInT2 = pOut;

      /* index used to check for pivot element */
      i = 0u;

      /* Loop over number of rows */
      /*  to be replaced by the sum of that row and a multiple of row i */
      k = numRows;

      while(k > 0u)
      {
        /* Check for the pivot element */
        if(i == l)
        {
          /* If the processing element is the pivot element,
             only the columns to the right are to be processed */
          pInT1 += numCols - l;

          pInT2 += numCols;
        }
        else
        {
          /* Element of the reference row */
          in = *pInT1;

          /* Working pointers for input and destination pivot rows */
          pPRT_in = pPivotRowIn;
          pPRT_pDst = pPivotRowDst;

          /* Loop over the number of columns to the right of the pivot element,
             to replace the elements in the input matrix */
          j = (numCols - l);

          while(j > 0u)
          {
            /* Replace the element by the sum of that row
               and a multiple of the reference row  */
            in1 = *pInT1;
            *pInT1++ = in1 - (in * *pPRT_in++);

            /* Decrement the loop counter */
            j--;
          }

          /* Loop over the number of columns to
             replace the elements in the destination matrix */
          j = numCols;

          while(j > 0u)
          {
            /* Replace the element by the sum of that row
               and a multiple of the reference row  */
            in1 = *pInT2;
            *pInT2++ = in1 - (in * *pPRT_pDst++);

            /* Decrement the loop counter */
            j--;
          }

        }

        /* Increment the temporary input pointer */
        pInT1 = pInT1 + l;

        /* Decrement the loop counter */
        k--;

        /* Increment the pivot index */
        i++;
      }

      /* Increment the input pointer */
      pIn++;

      /* Decrement the loop counter */
      loopCnt--;

      /* Increment the index modifier */
      l++;
    }


#else

  /* Run the below code for Cortex-M0 */

  float64_t Xchg, in = 0.0f;                     /* Temporary input values  */
  uint32_t i, rowCnt, flag = 0u, j, loopCnt, k, l;      /* loop counters */
  arm_status status;                             /* status of matrix inverse */

#ifdef ARM_MATH_MATRIX_CHECK

  /* Check for matrix mismatch condition */
  if((pSrc->numRows != pSrc->numCols) || (pDst->numRows != pDst->numCols)
     || (pSrc->numRows != pDst->numRows))
  {
    /* Set status as ARM_MATH_SIZE_MISMATCH */
    status = ARM_MATH_SIZE_MISMATCH;
  }
  else
#endif /*      #ifdef ARM_MATH_MATRIX_CHECK    */
  {

    /*--------------------------------------------------------------------------------------------------------------
	 * Matrix Inverse can be solved using elementary row operations.
	 *
	 *	Gauss-Jordan Method:
	 *
	 *	   1. First combine the identity matrix and the input matrix separated by a bar to form an
	 *        augmented matrix as follows:
	 *				        _  _	      _	    _	   _   _         _	       _
	 *					   |  |  a11  a12  | | | 1   0  |   |       |  X11 X12  |
	 *					   |  |            | | |        |   |   =   |           |
	 *					   |_ |_ a21  a22 _| | |_0   1 _|  _|       |_ X21 X21 _|
	 *
	 *		2. In our implementation, pDst Matrix is used as identity matrix.
	 *
	 *		3. Begin with the first row. Let i = 1.
	 *
	 *	    4. Check to see if the pivot for row i is zero.
	 *		   The pivot is the element of the main diagonal that is on the current row.
	 *		   For instance, if working with row i, then the pivot element is aii.
	 *		   If the pivot is zero, exchange that row with a row below it that does not
	 *		   contain a zero in column i. If this is not possible, then an inverse
	 *		   to that matrix does not exist.
	 *
	 *	    5. Divide every element of row i by the pivot.
	 *
	 *	    6. For every row below and  row i, replace that row with the sum of that row and
	 *		   a multiple of row i so that each new element in column i below row i is zero.
	 *
	 *	    7. Move to the next row and column and repeat steps 2 through 5 until you have zeros
	 *		   for every element below and above the main diagonal.
	 *
	 *		8. Now an identical matrix is formed to the left of the bar(input matrix, src).
	 *		   Therefore, the matrix to the right of the bar is our solution(dst matrix, dst).
	 *----------------------------------------------------------------------------------------------------------------*/

    /* Working pointer for destination matrix */
    pOutT1 = pOut;

    /* Loop over the number of rows */
    rowCnt = numRows;

    /* Making the destination matrix as identity matrix */
    while(rowCnt > 0u)
    {
      /* Writing all zeroes in lower triangle of the destination matrix */
      j = numRows - rowCnt;
      while(j > 0u)
      {
        *pOutT1++ = 0.0f;
        j--;
      }

      /* Writing all ones in the diagonal of the destination matrix */
      *pOutT1++ = 1.0f;

      /* Writing all zeroes in upper triangle of the destination matrix */
      j = rowCnt - 1u;
      while(j > 0u)
      {
        *pOutT1++ = 0.0f;
        j--;
      }

      /* Decrement the loop counter */
      rowCnt--;
    }

    /* Loop over the number of columns of the input matrix.
       All the elements in each column are processed by the row operations */
    loopCnt = numCols;

    /* Index modifier to navigate through the columns */
    l = 0u;
    //for(loopCnt = 0u; loopCnt < numCols; loopCnt++)
    while(loopCnt > 0u)
    {
      /* Check if the pivot element is zero..
       * If it is zero then interchange the row with non zero row below.
       * If there is no non zero element to replace in the rows below,
       * then the matrix is Singular. */

      /* Working pointer for the input matrix that points
       * to the pivot element of the particular row  */
      pInT1 = pIn + (l * numCols);

      /* Working pointer for the destination matrix that points
       * to the pivot element of the particular row  */
      pOutT1 = pOut + (l * numCols);

      /* Temporary variable to hold the pivot value */
      in = *pInT1;

      /* Destination pointer modifier */
      k = 1u;

      /* Check if the pivot element is zero */
      if(*pInT1 == 0.0f)
      {
        /* Loop over the number rows present below */
        for (i = (l + 1u); i < numRows; i++)
        {
          /* Update the input and destination pointers */
          pInT2 = pInT1 + (numCols * l);
          pOutT2 = pOutT1 + (numCols * k);

          /* Check if there is a non zero pivot element to
           * replace in the rows below */
          if(*pInT2 != 0.0f)
          {
            /* Loop over number of columns
             * to the right of the pilot element */
            for (j = 0u; j < (numCols - l); j++)
            {
              /* Exchange the row elements of the input matrix */
              Xchg = *pInT2;
              *pInT2++ = *pInT1;
              *pInT1++ = Xchg;
            }

            for (j = 0u; j < numCols; j++)
            {
              Xchg = *pOutT2;
              *pOutT2++ = *pOutT1;
              *pOutT1++ = Xchg;
            }

            /* Flag to indicate whether exchange is done or not */
            flag = 1u;

            /* Break after exchange is done */
            break;
          }

          /* Update the destination pointer modifier */
          k++;
        }
      }

      /* Update the status if the matrix is singular */
      if((flag != 1u) && (in == 0.0f))
      {
        return ARM_MATH_SINGULAR;
      }

      /* Points to the pivot row of input and destination matrices */
      pPivotRowIn = pIn + (l * numCols);
      pPivotRowDst = pOut + (l * numCols);

      /* Temporary pointers to the pivot row pointers */
      pInT1 = pPivotRowIn;
      pOutT1 = pPivotRowDst;

      /* Pivot element of the row */
      in = *(pIn + (l * numCols));

      /* Loop over number of columns
       * to the right of the pilot element */
      for (j = 0u; j < (numCols - l); j++)
      {
        /* Divide each element of the row of the input matrix
         * by the pivot element */
        *pInT1 = *pInT1 / in;
        pInT1++;
      }
      for (j = 0u; j < numCols; j++)
      {
        /* Divide each element of the row of the destination matrix
         * by the pivot element */
        *pOutT1 = *pOutT1 / in;
        pOutT1++;
      }

      /* Replace the rows with the sum of that row and a multiple of row i
       * so that each new element in column i above row i is zero.*/

      /* Temporary pointers for input and destination matrices */
      pInT1 = pIn;
      pOutT1 = pOut;

      for (i = 0u; i < numRows; i++)
      {
        /* Check for the pivot element */
        if(i == l)
        {
          /* If the processing element is the pivot element,
             only the columns to the right are to be processed */
          pInT1 += numCols - l;
          pOutT1 += numCols;
        }
        else
        {
          /* Element of the reference row */
          in = *pInT1;

          /* Working pointers for input and destination pivot rows */
          pPRT_in = pPivotRowIn;
          pPRT_pDst = pPivotRowDst;

          /* Loop over the number of columns to the right of the pivot element,
             to replace the elements in the input matrix */
          for (j = 0u; j < (numCols - l); j++)
          {
            /* Replace the element by the sum of that row
               and a multiple of the reference row  */
            *pInT1 = *pInT1 - (in * *pPRT_in++);
            pInT1++;
          }
          /* Loop over the number of columns to
             replace the elements in the destination matrix */
          for (j = 0u; j < numCols; j++)
          {
            /* Replace the element by the sum of that row
               and a multiple of the reference row  */
            *pOutT1 = *pOutT1 - (in * *pPRT_pDst++);
            pOutT1++;
          }

        }
        /* Increment the temporary input pointer */
        pInT1 = pInT1 + l;
      }
      /* Increment the input pointer */
      pIn++;

      /* Decrement the loop counter */
      loopCnt--;
      /* Increment the index modifier */
      l++;
    }


#endif /* #ifndef ARM_MATH_CM0_FAMILY */

    /* Set status as ARM_MATH_SUCCESS */
    status = ARM_MATH_SUCCESS;

    if((flag != 1u) && (in == 0.0f))
    {
      pIn = pSrc->pData;
      for (i = 0; i < numRows * numCols; i++)
      {
        if (pIn[i] != 0.0f)
            break;
      }

      if (i == numRows * numCols)
        status = ARM_MATH_SINGULAR;
    }
  }
  /* Return to application */
  return (status);
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
	float32_t KH[7][7] = {{0},{0}};
	float32_t KHP[7][7] = {{0},{0}};
	float32_t P2[7][7] = {{0},{0}};
	float32_t eye7[7][7] = {{0},{0}};

	int i;
	for (i = 0; i < 7; i++)
		eye7[i][i] = 1.0f;

	arm_matrix_instance_f32 P_f32_m;
	arm_matrix_instance_f32 P2_f32_m;
	arm_matrix_instance_f32 H_f32_m;
	arm_matrix_instance_f32 K_f32_m;
	arm_matrix_instance_f32 KH_f32_m;
	arm_matrix_instance_f32 KHP_f32_m;
	arm_matrix_instance_f32 eye7_f32_m;

	arm_mat_init_f32(&P_f32_m, 7, 7, &P[0][0]);
	arm_mat_init_f32(&P2_f32_m, 7, 7, &P2[0][0]);
	arm_mat_init_f32(&K_f32_m, 7, 6, &K[0][0]);
	arm_mat_init_f32(&H_f32_m, 6, 7, &H[0][0]);
	arm_mat_init_f32(&KH_f32_m, 7, 7, &KH[0][0]);
	arm_mat_init_f32(&KHP_f32_m, 7, 7, &KHP[0][0]);
	arm_mat_init_f32(&eye7_f32_m, 7, 7, &eye7[0][0]);

	arm_mat_mult_f32(&K_f32_m, &H_f32_m, &KH_f32_m);
	arm_mat_mult_f32(&KH_f32_m, &P_f32_m, &KHP_f32_m);
	arm_mat_sub_f32(&P_f32_m, &KHP_f32_m, &P2_f32_m);

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

void normalise3(float32_t r[3]){
	float32_t norm = sqrt(r[0]*r[0]+r[1]*r[1]+r[2]*r[2]);
	r[0] /= norm;
	r[1] /= norm;
	r[2] /= norm;
}

void triadComputation(float32_t r1_wf[3], float32_t r2_wf[3], float32_t r1_bf[3], float32_t r2_bf[3], float32_t ypr[3]){
	// find orthogonal vectors in both frame
	float32_t r3_wf[3];
	float32_t r13_wf[3];
	float32_t r3_bf[3];
	float32_t r13_bf[3];
	float32_t Rw2b[3][3];

	normalise3(r1_wf);
	normalise3(r2_wf);
	normalise3(r1_bf);
	normalise3(r2_bf);

	crossProduct3(r1_wf, r2_wf, r3_wf);
	crossProduct3(r1_wf, r3_wf, r13_wf);	// orthogonal set r1 r13 r3
	crossProduct3(r1_bf, r2_bf, r3_bf);
	crossProduct3(r1_bf, r3_bf, r13_bf);
	// compute rotation matrix

	float32_t bf[3][3];
	float32_t wf[3][3];
	float32_t wft[3][3];

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

void crossProduct3(float32_t a[3], float32_t b[3], float32_t c[3]) {
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}
