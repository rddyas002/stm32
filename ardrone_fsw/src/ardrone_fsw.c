
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
#include "motors.h"
#include "spektrum.h"

static volatile int keepRunning = 1;
void intHandler(int dummy) {
    keepRunning = 0;
}

extern struct navdata_t navdata;

int main(void) {
	signal(SIGINT, intHandler);
	//if (!navdata_init()) exit(1);
	//if(!actuators_ardrone_init()) exit(1);
	if(!spektrum_init()) exit(1);

	while(keepRunning){
		printf("Motor on\r\n");
		actuators_ardrone_set_power(0.2, 0, 0, 0);
		sleep(5);
		printf("Motor off\r\n");
		actuators_stop();
		sleep(5);

		/*
		actuators_ardrone_set_leds(MOT_LEDRED, MOT_LEDRED, MOT_LEDRED, MOT_LEDRED);
		usleep(50000);
		actuators_ardrone_set_leds(MOT_LEDORANGE, MOT_LEDORANGE, MOT_LEDORANGE, MOT_LEDORANGE);
		usleep(50000);
		actuators_ardrone_set_leds(MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN);
		usleep(50000);
*/
/*
 	 * navdata_update();
 * 		printf("%7d,%7d,%7d|%7d,%7d,%7d|%7d,%7d,%7d|%u|%u,%u\n",
				navdata.measure.vx, navdata.measure.vy, navdata.measure.vz,
				navdata.measure.ax, navdata.measure.ay, navdata.measure.az,
				navdata.measure.mx, navdata.measure.my, navdata.measure.mz,
				navdata.measure.ultrasound, navdata.measure.temperature_gyro, navdata.measure.temperature_acc);
				*/
	}
	printf("Terminated cleanly.\r\n");
	actuators_ardrone_close();
	return EXIT_SUCCESS;
}

