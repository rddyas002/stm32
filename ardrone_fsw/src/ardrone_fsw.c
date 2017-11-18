
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

#include "navdata.h"

struct navdata_t navdata;
static uint8_t navdata_buffer[NAVDATA_PACKET_SIZE];
static bool navdata_available = false;

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

#define GPIO_MAGIC 'p'
#define GPIO_DIRECTION _IOW(GPIO_MAGIC, 0, struct gpio_direction)
#define GPIO_READ _IOWR(GPIO_MAGIC, 1, struct gpio_data)
#define GPIO_WRITE _IOW(GPIO_MAGIC, 2, struct gpio_data)
int gpiofp = 0;

struct gpio_data {
  int pin;
  int value;
};

enum gpio_mode {
  GPIO_INPUT = 0,             //!< Pin configured for input
  GPIO_OUTPUT_LOW,            //!< Pin configured for output with low level
  GPIO_OUTPUT_HIGH,           //!< Pin configured for output with high level
};

struct gpio_direction {
  int pin;
  enum gpio_mode mode;
};

void gpio_setup_output(uint32_t port, uint16_t pin)
{
  /*
    if (port != 0x32524) return;  // protect ardrone board from unauthorized use
    struct gpio_direction dir;
    // Open the device if not open
    if (gpiofp == 0)
    gpiofp = open("/dev/gpio",O_RDWR);

    // Read the GPIO value
    dir.pin = pin;
    dir.mode = GPIO_OUTPUT_LOW;
    ioctl(gpiofp, GPIO_DIRECTION, &dir);
  */
}

void gpio_set(uint32_t port, uint16_t pin)
{
  if (port != 0x32524) { return; }  /* protect ardrone board from unauthorized use */
  struct gpio_data data;
  // Open the device if not open
  if (gpiofp == 0) {
    gpiofp = open("/dev/gpio", O_RDWR);
  }

  /* Read the GPIO value */
  data.pin = pin;
  data.value = 1;
  ioctl(gpiofp, GPIO_WRITE, &data);
}

/**
 * Sends a one byte command
 */
static void navdata_cmd_send(uint8_t cmd)
{
  full_write(navdata.fd, &cmd, 1);
}


int main(void) {
	printf("!!!Hello From Drone!!!\r\n");

	/* Check if the FD isn't already initialized */
	if (navdata.fd <= 0) {
		navdata.fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY); /* O_NONBLOCK doesn't work */

		if (navdata.fd < 0) {
			printf("[navdata] Unable to open navdata board connection(/dev/ttyO1)\n");
			exit(1);
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

	int newbytes = read(navdata.fd, navdata_buffer, NAVDATA_PACKET_SIZE);

	printf("bytes received: %d\r\n", newbytes);
	int i;
	for (i = 0; i < newbytes; i++){
		printf("0x%.2X ", navdata_buffer[i]);
	}

	return EXIT_SUCCESS;
}
