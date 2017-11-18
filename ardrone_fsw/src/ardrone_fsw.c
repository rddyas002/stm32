
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>   /* for baud rates and options */
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <signal.h>

#include "navdata.h"
#include "gpio.h"

static volatile int keepRunning = 1;
void intHandler(int dummy) {
    keepRunning = 0;
}

extern struct navdata_t navdata;

int main(void) {
	signal(SIGINT, intHandler);

	printf("!!!Hello From Drone!!!\r\n");

	while(keepRunning){
		navdata_update();
		printf("%7d,%7d,%7d\n", navdata.measure.vx, navdata.measure.vy, navdata.measure.vz);
	}

	return EXIT_SUCCESS;
}

