
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
	//if(!actuators_ardrone_init()) exit(1);
	//if(!spektrum_init()) exit(1);

	while(keepRunning){
		usleep(20000);
		//actuators_ardrone_set_power(getThrottle(), 0, 0, 0);


/*
		actuators_ardrone_set_pwm(20, 0, 0, 0);
		usleep(100000);
		actuators_ardrone_set_pwm(0, 0, 0, 0);
		usleep(100000);
		*/
		//actuators_ardrone_set_power(getThrottle(), 0, 0, 0);
	/*
		printf("Motor on\r\n");
		actuators_ardrone_set_power(0.2, 0, 0, 0);
		sleep(1);
		printf("Motor off\r\n");
		actuators_stop();
		sleep(1);
*/
/*
		printf("%7d,%7d,%7d,%7d,%7d,%7d,%7d,%7d,%7d,%7u,%7u,%7u\n",
				navdata.measure.vx, navdata.measure.vy, navdata.measure.vz,
				navdata.measure.ax, navdata.measure.ay, navdata.measure.az,
				navdata.measure.mx, navdata.measure.my, navdata.measure.mz,
				navdata.measure.ultrasound, navdata.measure.temperature_gyro, navdata.measure.temperature_acc);
				*/
	}
	//actuators_ardrone_close();
	close_navdata();
	//close_spektrum();
	printf("Terminated cleanly.\r\n");
	return EXIT_SUCCESS;
}

