
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

	}
	//actuators_ardrone_close();
	close_navdata();
	//close_spektrum();
	printf("Terminated cleanly.\r\n");
	return EXIT_SUCCESS;
}

