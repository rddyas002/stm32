#include <math.h>
#include "timing.h"


double timeNow_us(void){
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return floor((tv.tv_sec)*1e6 + (tv.tv_nsec)/1e3 + 0.5);
}

double timeSince(double time_t0){
	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	return ((tv.tv_sec)*1e6 + (tv.tv_nsec)/1e3 - time_t0);
}
