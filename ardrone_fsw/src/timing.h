/*
 * timing.h
 *
 *  Created on: 20 Nov 2017
 *      Author: yreddi
 */

#ifndef TIMING_H_
#define TIMING_H_

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>

double timeNow_us(void);
double timeSince(double time_t0);

#endif /* TIMING_H_ */
