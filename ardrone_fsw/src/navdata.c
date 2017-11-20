#include "navdata.h"
#include "gpio.h"

struct navdata_t navdata;
static uint8_t navdata_buffer[NAVDATA_PACKET_SIZE];
static bool navdata_available = false;

/* syncronization variables */
static pthread_mutex_t navdata_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  navdata_cond  = PTHREAD_COND_INITIALIZER;

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


/**
 * Main reading thread
 * This is done asynchronous because the navdata board doesn't support NON_BLOCKING
 */
static void *navdata_read(void *data __attribute__((unused)))
{
	/* Buffer insert index for reading/writing */
	static uint8_t buffer_idx = 0;

	while (true) {

		/* Wait until we are notified to read next data,
       i.e. buffer has been copied in navdata_update */
		pthread_mutex_lock(&navdata_mutex);
		while (navdata_available) {
			navdata_available = false;

		printf("%7d,%7d,%7d|%7d,%7d,%7d|%7d,%7d,%7d|%u|%u,%u\n",
				navdata.measure.vx, navdata.measure.vy, navdata.measure.vz,
				navdata.measure.ax, navdata.measure.ay, navdata.measure.az,
				navdata.measure.mx, navdata.measure.my, navdata.measure.mz,
				navdata.measure.ultrasound, navdata.measure.temperature_gyro, navdata.measure.temperature_acc);

			pthread_cond_wait(&navdata_cond, &navdata_mutex);
		}
		pthread_mutex_unlock(&navdata_mutex);

		/* Read new bytes */
		int newbytes = read(navdata.fd, &navdata_buffer[0] + buffer_idx, NAVDATA_PACKET_SIZE - buffer_idx);


		// When there was no signal interrupt
		if (newbytes > 0) {
			buffer_idx += newbytes;
			navdata.totalBytesRead += newbytes;
		}

		// If we got a full packet
		if (buffer_idx >= NAVDATA_PACKET_SIZE) {
			// check if the start byte is correct
			if (navdata_buffer[0] != NAVDATA_START_BYTE) {
				uint8_t *pint = memchr(navdata_buffer, NAVDATA_START_BYTE, buffer_idx);

				// Check if we found the start byte in the read data
				if (pint != NULL) {
					memmove(navdata_buffer, pint, NAVDATA_PACKET_SIZE - (pint - navdata_buffer));
					buffer_idx = pint - navdata_buffer;
					fprintf(stderr, "[navdata] sync error, startbyte not found, resetting...\n");
				} else {
					buffer_idx = 0;
				}
				continue;
			}

			// full packet read with startbyte at the beginning, reset insert index
			buffer_idx = 0;

			// Calculate the checksum
			uint16_t checksum = 0;
			int i;
			for (i = 2; i < NAVDATA_PACKET_SIZE - 2; i += 2) {
				checksum += navdata_buffer[i] + (navdata_buffer[i + 1] << 8);
			}

			struct navdata_measure_t *new_measurement = (struct navdata_measure_t *)navdata_buffer;

			// Check if the checksum is OK
			if (new_measurement->chksum != checksum) {
				fprintf(stderr, "[navdata] Checksum error [calculated: %d] [packet: %d] [diff: %d]\n",
						checksum, new_measurement->chksum, checksum - new_measurement->chksum);
				navdata.checksum_errors++;

				continue;
			}

			// Set flag that we have new valid navdata
			pthread_mutex_lock(&navdata_mutex);
			navdata_available = true;
			pthread_mutex_unlock(&navdata_mutex);
		}
	}

	return NULL;
}


bool navdata_init(void){
	/* Check if the FD isn't already initialized */
	if (navdata.fd <= 0) {
		navdata.fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY); /* O_NONBLOCK doesn't work */

		if (navdata.fd < 0) {
			printf("[navdata] Unable to open navdata board connection(/dev/ttyO1)\n");
			return false;
		}

		/* Update the settings of the UART connection */
		fcntl(navdata.fd, F_SETFL, 0); /* read calls are non blocking */
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

	/* Start navdata reading thread */
	pthread_t navdata_thread;
	if (pthread_create(&navdata_thread, NULL, navdata_read, NULL) != 0) {
		printf("[navdata] Could not create navdata reading thread!\n");
		return false;
	}
	printf("[navdata] Read thread started!\n");

	return true;
}

/**
 * Update the navdata (event loop)
 */
void navdata_update()
{
	pthread_mutex_lock(&navdata_mutex);
	/* If we got a new navdata packet */
	if (navdata_available) {

		/* Copy the navdata packet */
		memcpy(&navdata.measure, navdata_buffer, NAVDATA_PACKET_SIZE);

		/* reset the flag */
		navdata_available = false;
		/* signal that we copied the buffer and new packet can be read */
		pthread_cond_signal(&navdata_cond);
		pthread_mutex_unlock(&navdata_mutex);

		/* Check if we missed a packet (our counter and the one from the navdata) */
		navdata.last_packet_number++;
		if (navdata.last_packet_number != navdata.measure.nu_trame) {
			fprintf(stderr, "[navdata] Lost frame: %d should have been %d\n",
					navdata.measure.nu_trame, navdata.last_packet_number);
			navdata.lost_imu_frames++;
		}
		navdata.last_packet_number = navdata.measure.nu_trame;

	    /* Invert byte order so that TELEMETRY works better */
	    uint8_t tmp;
	    uint8_t *p = (uint8_t *) & (navdata.measure.pressure);
	    tmp = p[0];
	    p[0] = p[1];
	    p[1] = tmp;
	    p = (uint8_t *) & (navdata.measure.temperature_pressure);
	    tmp = p[0];
	    p[0] = p[1];
	    p[1] = tmp;

		navdata.packetsRead++;
	} else {
		/* no new packet available, still unlock mutex again */
		pthread_mutex_unlock(&navdata_mutex);
	}
}
