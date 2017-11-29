#ifndef MATRIX_MATH_H_
#define MATRIX_MATH_H_

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

void transpose32(float * a, float *  at, int r, int c);
void mul32m(float * a, float * b, float * c, int arows, int acols, int bcols);
void accum32m(float * a, float * b, int m, int n);
void add32m(float * a, float * b, float * c, int n, int m);
void add32v(float * u, float * v, float * w, int n);
void crossProduct3_32(float a[3], float b[3], float c[3]);
void sub32m(float * a, float * b, float * c, int n, int m);
void sub32v(float * u, float * v, float * w, int n);
void normalise32(float v[], int n);
void copy32v(float * dst, float * src, int n);
void copy32m(float * dst, float * src, int n, int m);

int choldc1(float * a, float * p, int n);
int choldcsl(float * A, float * a, float * p, int n);
int cholsl(float * A, float * a, float * p, int n);

#endif /* MATRIX_MATH_H_ */
