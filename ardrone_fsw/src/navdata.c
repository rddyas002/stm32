#include "navdata.h"
#include "gpio.h"
#include "timing.h"
#include "imu_ardrone2.h"
#include "ekf.h"

/** Sonar offset.
 *  Offset value in ADC
 *  equals to the ADC value so that height is zero
 */
#ifndef SONAR_OFFSET
#define SONAR_OFFSET 880
#endif

/** Sonar scale.
 *  Sensor sensitivity in m/adc (float)
 */
#ifndef SONAR_SCALE
#define SONAR_SCALE 0.00047
#endif


struct navdata_t navdata;
static uint8_t navdata_buffer[NAVDATA_PACKET_SIZE];
volatile static bool navdata_available = false;
volatile static bool navdata_bytes_available = false;
volatile bool nav_initialisation = false;
volatile uint32_t average_counter = 0;
volatile double gyro_rate_bias[3] = {0};
volatile double accel_bias[3] = {0};
volatile double mag_bias[3] = {0};
double nav_start_time = 0;
imu_data_s imu_data;
float b_32[3] = {0};
float q_32[4] = {0};
float ypr_32[3] = {0};

/* syncronization variables */
static pthread_mutex_t navdata_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  navdata_cond  = PTHREAD_COND_INITIALIZER;
pthread_t navdata_thread;

#ifdef NAVDATA_LOGGING
FILE *navdata_file_p;
#endif

void print_navdata(void);
double getNavTime(void);
float normalise3v(float v[3]);

void signal_handler_navdata (int status){
	ioctl(navdata.fd, FIONREAD, &navdata_bytes_available);
	if (navdata_bytes_available >= 60)
		pthread_cond_signal(&navdata_cond);
}

ssize_t full_write(int fd, const uint8_t *buf, size_t count)
{
	size_t written = 0;

	while (written < count) {
		ssize_t n = write(fd, buf + written, count - written);
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				continue;
			}
			return n;
		}
		written += n;
	}
	return written;
}

/**
 * Read from fd even while being interrupted
 */
ssize_t full_read(int fd, uint8_t *buf, size_t count)
{
	/* Apologies for illiteracy, but we can't overload |read|.*/
	size_t readed = 0;

	while (readed < count) {
		ssize_t n = read(fd, buf + readed, count - readed);
		if (n < 0) {
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				continue;
			}
			return n;
		}
		readed += n;
	}
	return readed;
}

/**
 * Sends a one byte command
 */
static void navdata_cmd_send(uint8_t cmd)
{
	full_write(navdata.fd, &cmd, 1);
}

float computeMean(float m, float x, float k){
	return m + (x - m)/k;
}

float computeVariance(float v, float x, float m, float m_k){
	return v + (x - m)*(x - m_k);
}

void computeGyroStats(imu_data_s * imu_data){
	static float m_g[3] = {0};
	static float v_g[3] = {0};
	static float m_a[3] = {0};
	static float v_a[3] = {0};
	static float m_m[3] = {0};
	static float v_m[3] = {0};

	static uint32_t count = 0;
	static bool first_enter = true;
	int i;

	if (first_enter){
		for (i = 0; i < 3; i++){
			v_g[i] = 0.0f;
			m_g[i] = imu_data->rate[i];
			v_a[i] = 0.0f;
			m_a[i] = imu_data->acceleration[i];
			v_m[i] = 0.0f;
			m_m[i] = imu_data->magnetic[i];
		}

		count++;
		first_enter = false;
		return;
	}

	float m_k;
	for (i = 0; i < 3; i++){
		m_k = computeMean(m_g[i], imu_data->rate[i], (float)count);
		v_g[i] = computeVariance(v_g[i], imu_data->rate[i], m_g[i], m_k);
		m_g[i] = m_k;

		m_k = computeMean(m_a[i], imu_data->acceleration[i], (float)count);
		v_a[i] = computeVariance(v_a[i], imu_data->acceleration[i], m_a[i], m_k);
		m_a[i] = m_k;

		m_k = computeMean(m_m[i], imu_data->magnetic[i], (float)count);
		v_m[i] = computeVariance(v_m[i], imu_data->magnetic[i], m_m[i], m_k);
		m_m[i] = m_k;

		count++;
		imu_data->gyro_var[i] = v_g[i]/((float)count - 1.0f);
		imu_data->accel_var[i] = v_a[i]/((float)count - 1.0f);
		imu_data->mag_var[i] = v_m[i]/((float)count - 1.0f);
	}
}


/**
 * Main reading thread
 * This is done asynchronous because the navdata board doesn't support NON_BLOCKING
 */
static void *navdata_read(void *data __attribute__((unused))){
	double prev_time = 0;

	printf("[navdata] Read thread started!\n");

	while (true) {
		pthread_mutex_lock(&navdata_mutex);
		pthread_cond_wait(&navdata_cond, &navdata_mutex);
		pthread_mutex_unlock(&navdata_mutex);
		// 60 bytes are available
		int newbytes = read(navdata.fd, &navdata_buffer[0], NAVDATA_PACKET_SIZE);

		if ((navdata_buffer[0] == NAVDATA_START_BYTE) && (navdata_buffer[1] == NAVDATA_SECOND_BYTE)){
			// Calculate the checksum
			uint16_t checksum = 0;
			int i;
			for (i = 2; i < NAVDATA_PACKET_SIZE - 2; i += 2) {
				checksum += navdata_buffer[i] + (navdata_buffer[i + 1] << 8);
			}

			struct navdata_measure_t *new_measurement = (struct navdata_measure_t *)navdata_buffer;
			if (checksum == new_measurement->chksum){
				pthread_mutex_lock(&navdata_mutex);
				memcpy(&navdata.measure, new_measurement, sizeof(struct navdata_measure_t));
				// scale measurments
				imu_data.rate[0] = (float)((navdata.measure.vx)*IMU_GYRO_P_SIGN) / IMU_GYRO_SENS;
				imu_data.rate[1] = (float)((navdata.measure.vy)*IMU_GYRO_Q_SIGN) / IMU_GYRO_SENS;
				imu_data.rate[2] = (float)((navdata.measure.vz)*IMU_GYRO_R_SIGN) / IMU_GYRO_SENS;
				imu_data.rate[0] *= M_PI_f/180.0f;
				imu_data.rate[1] *= M_PI_f/180.0f;
				imu_data.rate[2] *= M_PI_f/180.0f;
				imu_data.acceleration[0] = (float)(navdata.measure.ax - IMU_ACCEL_X_NEUTRAL)*IMU_ACCEL_X_SIGN / IMU_ACCEL_SENS;
				imu_data.acceleration[1] = (float)(navdata.measure.ay - IMU_ACCEL_Y_NEUTRAL)*IMU_ACCEL_Y_SIGN / IMU_ACCEL_SENS;
				imu_data.acceleration[2] = (float)(navdata.measure.az - IMU_ACCEL_Z_NEUTRAL)*IMU_ACCEL_Z_SIGN / IMU_ACCEL_SENS;
				imu_data.magnetic[0] = (float)(navdata.measure.mx - IMU_MAG_X_OFFSET)*IMU_MAG_X_SIGN;
				imu_data.magnetic[1] = (float)(navdata.measure.my - IMU_MAG_Y_OFFSET)*IMU_MAG_Y_SIGN;
				imu_data.magnetic[2] = (float)(navdata.measure.mz - IMU_MAG_Z_OFFSET)*IMU_MAG_Z_SIGN;
				double time_now = getNavTime();
				imu_data.time = (float) time_now;

				// normalise mag and accel
				float magnetic_norm = normalise3v(imu_data.magnetic);
				float accel_norm = normalise3v(imu_data.acceleration);

				if (nav_initialisation){
					average_counter++;
					gyro_rate_bias[0] += (double)imu_data.rate[0];
					gyro_rate_bias[1] += (double)imu_data.rate[1];
					gyro_rate_bias[2] += (double)imu_data.rate[2];
					accel_bias[0] += (double)imu_data.acceleration[0];
					accel_bias[1] += (double)imu_data.acceleration[1];
					accel_bias[2] += (double)imu_data.acceleration[2];
					mag_bias[0] += (double)imu_data.magnetic[0];
					mag_bias[1] += (double)imu_data.magnetic[1];
					mag_bias[2] += (double)imu_data.magnetic[2];
					computeGyroStats(&imu_data);
				}

				if(imu_data.initialised){
					if (imu_data.Ts == 0){
						imu_data.Ts = 5e-3;
					}
					else{
						imu_data.Ts = (float)(time_now - prev_time)/1e6;
					}
					prev_time = time_now;
					run_ekf(&imu_data);
					q2ypr(imu_data.q, imu_data.ypr);
				}
#ifdef PRINT_NAV_DEBUG
				print_navdata();
#endif
#ifdef NAVDATA_LOGGING
				fprintf(navdata_file_p,"%.0f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
						getNavTime(),
						imu_data.rate[0],imu_data.rate[1],imu_data.rate[2],
						imu_data.acceleration[0],imu_data.acceleration[1],imu_data.acceleration[2],
						imu_data.magnetic[0],imu_data.magnetic[1],imu_data.magnetic[2]);
#endif

				navdata_available = true;
				pthread_mutex_unlock(&navdata_mutex);
			}
		}
		else{
			// Flush input buffer until syncd
			tcflush(navdata.fd, TCIFLUSH);
		}
	}

	return NULL;
}

void getEstimates(float * ypr, float *w_bias){
	ypr = &imu_data.ypr[0];
	w_bias = &imu_data.w_bias[0];
}


bool navdata_init(void){
	struct sigaction saio;

	printf("Navdata init\n");
	imu_data.Ts = 0;
	imu_data.initialised = false;
	nav_start_time = timeNow_us();

	/* Check if the FD isn't already initialized */
	if (navdata.fd <= 0) {
		navdata.fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY); /* O_NONBLOCK doesn't work */

		if (navdata.fd < 0) {
			printf("[navdata] Unable to open navdata board connection(/dev/ttyO1)\n");
			return false;
		}

		// install the signal handler before making the device asynchronous
		saio.sa_handler = signal_handler_navdata;
		sigemptyset(&saio.sa_mask);
		saio.sa_flags = 0;
		saio.sa_restorer = NULL;
		sigaction(SIGIO,&saio,NULL);

		// allow the process to receive SIGIO
		fcntl(navdata.fd, F_SETOWN, getpid());
		fcntl(navdata.fd, F_SETFL, FASYNC);

		/* Update the settings of the UART connection */
		//	fcntl(navdata.fd, F_SETFL, 0); /* read calls are non blocking */
		/* set port options */
		struct termios options;
		/* Get the current options for the port */
		tcgetattr(navdata.fd, &options);
		/* Set the baud rates to 460800 */
		cfsetispeed(&options, B460800);
		cfsetospeed(&options, B460800);

		options.c_cflag |= (CLOCAL | CREAD); /* Enable the receiver and set local mode */
		options.c_iflag = 0; /* clear input options */
		options.c_lflag = 0; /* clear local options */
		options.c_oflag &= ~OPOST; //clear output options (raw output)

		//Set the new options for the port
		tcsetattr(navdata.fd, TCSANOW, &options);
	}

	// Reset available flags
	navdata_available = false;
	navdata.imu_lost = false;

	// Set all statistics to 0
	navdata.checksum_errors = 0;
	navdata.lost_imu_frames = 0;
	navdata.totalBytesRead = 0;
	navdata.packetsRead = 0;
	navdata.last_packet_number = 0;

	/* Stop acquisition */
	navdata_cmd_send(NAVDATA_CMD_STOP);

	/* Start acquisition */
	navdata_cmd_send(NAVDATA_CMD_START);

	/* Set navboard gpio control */
	gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);
	gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_NAVDATA);

	nav_initialisation = true;
	/* Start navdata reading thread */
	if (pthread_create(&navdata_thread, NULL, navdata_read, NULL) != 0) {
		printf("[navdata] Could not create navdata reading thread!\n");
		return false;
	}

#ifdef NAVDATA_LOGGING
	// open file for logging
	navdata_file_p = fopen(NAVDATA_LOGFILE, "w");
	if (navdata_file_p == NULL) {
		fprintf(stderr, "[navdata]Can't open output file!\n");
		return false;
	}
#endif

	// capture data for 3 seconds
	printf("NAV data capturing data for mean estimation.\n");
	while(getNavTime() < 3000000);
	pthread_mutex_lock(&navdata_mutex);
	nav_initialisation = false;
	pthread_mutex_unlock(&navdata_mutex);
	imu_data.gyro_offset[0] = (float)(gyro_rate_bias[0])/average_counter;
	imu_data.gyro_offset[1] = (float)(gyro_rate_bias[1])/average_counter;
	imu_data.gyro_offset[2] = (float)(gyro_rate_bias[2])/average_counter;
	imu_data.accel_offset[0] = (float)accel_bias[0]/average_counter;
	imu_data.accel_offset[1] = (float)accel_bias[1]/average_counter;
	imu_data.accel_offset[2] = (float)accel_bias[2]/average_counter;
	imu_data.mag_offset[0] = (float)mag_bias[0]/average_counter;
	imu_data.mag_offset[1] = (float)mag_bias[1]/average_counter;
	imu_data.mag_offset[2] = (float)mag_bias[2]/average_counter;
	printf("[NAVDATA] Gyro average: %.2f,%.2f,%.2f\n", imu_data.gyro_offset[0]*180.0f/M_PI_f, imu_data.gyro_offset[1]*180.0f/M_PI_f, imu_data.gyro_offset[2]*180.0f/M_PI_f);
	printf("[NAVDATA] Gyro variance: %f,%f,%f\n", imu_data.gyro_var[0], imu_data.gyro_var[1], imu_data.gyro_var[2]);
	printf("[NAVDATA] Accel average: %.2f,%.2f,%.2f\n", imu_data.accel_offset[0], imu_data.accel_offset[1], imu_data.accel_offset[2]);
	printf("[NAVDATA] Accel variance: %f,%f,%f\n", imu_data.accel_var[0], imu_data.accel_var[1], imu_data.accel_var[2]);
	printf("[NAVDATA] Mag average: %.2f,%.2f,%.2f\n", imu_data.mag_offset[0], imu_data.mag_offset[1], imu_data.mag_offset[2]);
	printf("[NAVDATA] Mag variance: %f,%f,%f\n", imu_data.mag_var[0], imu_data.mag_var[1], imu_data.mag_var[2]);
	init_ekf(&imu_data);
	pthread_mutex_lock(&navdata_mutex);
	imu_data.initialised = true;
	pthread_mutex_unlock(&navdata_mutex);
	return true;
}

double getNavTime(void){
	return timeNow_us() - nav_start_time;
}

float normalise3v(float v[3]){
	float norm = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
	norm = sqrt(norm);
	v[0] /= norm;
	v[1] /= norm;
	v[2] /= norm;
	return norm;
}

void print_navdata(void){
	printf("%.0f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f,%6.2f\n",
			getNavTime(),
			imu_data.rate[0],imu_data.rate[1],imu_data.rate[2],
			imu_data.acceleration[0],imu_data.acceleration[1],imu_data.acceleration[2],
			imu_data.magnetic[0],imu_data.magnetic[1],imu_data.magnetic[2]);
}

void close_navdata(void){
	if (navdata.fd != NULL)
		close(navdata.fd);
#ifdef NAVDATA_LOGGING
	if (navdata_file_p != NULL)
		close(navdata_file_p);
#endif
}
