
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
	if (!navdata_init()) exit(1);

	while(keepRunning){
/*		usleep(100000);
		navdata_update();
		printf("%7d,%7d,%7d|%7d,%7d,%7d|%7d,%7d,%7d|%u|%u,%u\n",
				navdata.measure.vx, navdata.measure.vy, navdata.measure.vz,
				navdata.measure.ax, navdata.measure.ay, navdata.measure.az,
				navdata.measure.mx, navdata.measure.my, navdata.measure.mz,
				navdata.measure.ultrasound, navdata.measure.temperature_gyro, navdata.measure.temperature_acc);
				*/
	}
	printf("Terminated cleanly.\r\n");

	return EXIT_SUCCESS;
}

