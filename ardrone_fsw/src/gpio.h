/*
 * gpio.h
 *
 *  Created on: 18 Nov 2017
 *      Author: RDDYA
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <stdint.h>

#define GPIO_MAGIC 'p'
#define GPIO_DIRECTION _IOW(GPIO_MAGIC, 0, struct gpio_direction)
#define GPIO_READ _IOWR(GPIO_MAGIC, 1, struct gpio_data)
#define GPIO_WRITE _IOW(GPIO_MAGIC, 2, struct gpio_data)

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


void gpio_setup_output(uint32_t port, uint16_t pin);
void gpio_set(uint32_t port, uint16_t pin);

#endif /* GPIO_H_ */
