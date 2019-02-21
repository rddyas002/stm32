#include "matrix_math.h"

void transpose64(double * a, double * at, int r, int c){
    int i,j;
    for(i = 0; i < r; ++i){
        for(j = 0; j < c; ++j) {
        	at[j*r+i] = a[i*c+j];
        }
    }
}

void mul64m(double * a, double * b, double * c, int arows, int acols, int bcols){
    int i, j,l;
    for(i=0; i<arows; ++i)
        for(j=0; j<bcols; ++j) {
            c[i*bcols+j] = 0;
            for(l=0; l<acols; ++l)
                c[i*bcols+j] += a[i*acols+l] * b[l*bcols+j];
        }
}

void accum64m(double * a, double * b, int m, int n){
    int i,j;

    for(i=0; i<m; ++i)
        for(j=0; j<n; ++j)
            a[i*n+j] += b[i*n+j];
}

void add64m(double * a, double * b, double * c, int n, int m){
    int j;

    for(j=0; j<n*m; ++j)
        c[j] = a[j] + b[j];
}

void add64v(double * u, double * v, double * w, int n){
	for (int i = 0; i < n; i++)
		w[i] = u[i] + v[i];
}

void crossProduct3_64(double a[3], double b[3], double c[3]) {
	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];
}

void sub64m(double * a, double * b, double * c, int n, int m){
    int j;

    for(j=0; j<n*m; ++j)
        c[j] = a[j] - b[j];
}

void sub64v(double * u, double * v, double * w, int n){
	for (int i = 0; i < n; i++)
		w[i] = u[i] - v[i];
}

void normalise64(double * v, int n){
	int i;
	double norm = 0;
	for (i = 0; i < n; i++){
		norm += v[i]*v[i];
	}
	norm = sqrt(norm);
	for (i = 0; i < n; i++){
		v[i] /= norm;
	}
}

void copy64v(double * dst, double * src, int n){
	memcpy(dst, src, n*sizeof(double));
}

void copy64m(double * dst, double * src, int n, int m){
	memcpy(dst, src, n*m*sizeof(double));
}

int choldc1(double * a, double * p, int n) {
    int i,j,k;
    double sum;

    for (i = 0; i < n; i++) {
        for (j = i; j < n; j++) {
            sum = a[i*n+j];
            for (k = i - 1; k >= 0; k--) {
                sum -= a[i*n+k] * a[j*n+k];
            }
            if (i == j) {
                if (sum <= 0) {
                    return 1; /* error */
                }
                p[i] = sqrt(sum);
            }
            else {
                a[j*n+i] = sum / p[i];
            }
        }
    }

    return 0; /* success */
}

int choldcsl(double * A, double * a, double * p, int n)
{
    int i,j,k; double sum;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            a[i*n+j] = A[i*n+j];
    if (choldc1(a, p, n)) return 1;
    for (i = 0; i < n; i++) {
        a[i*n+i] = 1 / p[i];
        for (j = i + 1; j < n; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= a[j*n+k] * a[k*n+i];
            }
            a[j*n+i] = sum / p[j];
        }
    }

    return 0; /* success */
}


int cholsl(double * A, double * a, double * p, int n)
{
    int i,j,k;
    if (choldcsl(A,a,p,n)) return 1;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            a[i*n+j] = 0.0;
        }
    }
    for (i = 0; i < n; i++) {
        a[i*n+i] *= a[i*n+i];
        for (k = i + 1; k < n; k++) {
            a[i*n+i] += a[k*n+i] * a[k*n+i];
        }
        for (j = i + 1; j < n; j++) {
            for (k = j; k < n; k++) {
                a[i*n+j] += a[k*n+i] * a[k*n+j];
            }
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
            a[i*n+j] = a[j*n+i];
        }
    }

    return 0; /* success */
}
