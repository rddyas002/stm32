
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
#include "timing.h"

static volatile int keepRunning = 1;
void intHandler(int dummy) {
    keepRunning = 0;
}

extern struct navdata_t navdata;

int main(void) {
	signal(SIGINT, intHandler);
	double t0 = timeNow_us();

	if (!navdata_init()) exit(1);
//	if(!actuators_ardrone_init()) exit(1);
//	if(!spektrum_init()) exit(1);

	while(keepRunning){
		/*
		float throttle = getThrottle();
		float roll = getRoll();
		float pitch = getPitch();
		float yaw = getYaw();

		float m1 = throttle + 0.25*pitch + 0.25*roll + 0.25*yaw;
		float m2 = throttle + 0.25*pitch - 0.25*roll - 0.25*yaw;
		float m3 = throttle - 0.25*pitch + 0.25*roll + 0.25*yaw;
		float m4 = throttle - 0.25*pitch - 0.25*roll - 0.25*yaw;

		actuators_ardrone_set_power(m1, m2, m3, m4);
		actuators_ardrone_commit_color(MOT_LEDRED, MOT_LEDRED, MOT_LEDRED, MOT_LEDRED);
		usleep(20000);
		actuators_ardrone_set_power(m1, m2, m3, m4);
		actuators_ardrone_commit_color(MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN);
		usleep(20000);
		*/
	}
//	actuators_ardrone_close();
	close_navdata();
//	close_spektrum();
	printf("Terminated cleanly.\r\n");
	return EXIT_SUCCESS;
}

