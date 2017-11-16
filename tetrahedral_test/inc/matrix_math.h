#ifndef MATRIX_MATH_H_
#define MATRIX_MATH_H_

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

void transpose64(double * a, double *  at, int r, int c);
void mul64m(double * a, double * b, double * c, int arows, int acols, int bcols);
void accum64m(double * a, double * b, int m, int n);
void add64m(double * a, double * b, double * c, int n, int m);
void add64v(double * u, double * v, double * w, int n);
void crossProduct3_64(double a[3], double b[3], double c[3]);
void sub64m(double * a, double * b, double * c, int n, int m);
void sub64v(double * u, double * v, double * w, int n);
void normalise64(double v[], int n);
void copy64v(double * dst, double * src, int n);
void copy64m(double * dst, double * src, int n, int m);

int choldc1(double * a, double * p, int n);
int choldcsl(double * A, double * a, double * p, int n);
int cholsl(double * A, double * a, double * p, int n);

#endif /* MATRIX_MATH_H_ */
