
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
#include "ekf.h"

static volatile int keepRunning = 1;
void intHandler(int dummy) {
    keepRunning = 0;
}

extern struct navdata_t navdata;
extern imu_data_s imu_data;

int main(void) {
	signal(SIGINT, intHandler);
	if (!navdata_init()) exit(1);
	//if(!actuators_ardrone_init()) exit(1);
	//if(!spektrum_init()) exit(1);

	while(keepRunning){
		double now = timeNow_us();
		while ((timeNow_us() - now) < 1000000);
		printf("%5.2f,%5.2f,%5.2f|%5.2f,%5.2f,%5.2f\r\n", imu_data.ypr[0],imu_data.ypr[1],imu_data.ypr[2],imu_data.w_bias[0]*180.0f/M_PI_f,imu_data.w_bias[1]*180.0f/M_PI_f,imu_data.w_bias[2]*180.0f/M_PI_f);

	}
	//actuators_ardrone_close();
	close_navdata();
	//close_spektrum();
	printf("Terminated cleanly.\r\n");
	return EXIT_SUCCESS;
}

