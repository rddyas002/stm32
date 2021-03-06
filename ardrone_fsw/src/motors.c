/*
 * Original Code from:
 * Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com
 *
 * Adapated for Paparazzi by:
 * Copyright (C) 2012 Dino Hensen <dino.hensen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file boards/ardrone/actuators.c
 * Actuator driver for ardrone2 version
 */
#include "motors.h"
#include "gpio.h"
/*
#include "subsystems/actuators.h"
#include "actuators.h"
#include "mcu_periph/gpio.h"
#include "led_hw.h"
#include "mcu_periph/sys_time.h"
 */
#include "navdata.h" // for full_write

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdint.h>

#include "timing.h"

typedef struct {
	float power[4]; //motor speed setting. 0.0=min power, 1.0=full power
	uint16_t pwm[4];   //motor speed 0x00-0x1ff.  -- protected by mutex
	uint8_t led[4];   //led 0=off 1=red 2=green 3=orange -- protected by mutex
	uint8_t NeedToSendLedCmd;
}motor_t;

pthread_t mot_thread;
static pthread_mutex_t mot_mutex = PTHREAD_MUTEX_INITIALIZER;
motor_t motor;

/**
 * Power consumption @ 11V all 4 motors running
 * PWM   A
 * 0    0.2
 * 80   1.3
 * 100  1.5
 * 150  2.0
 * 190  2.5
 * 130  3.0
 */
int actuator_ardrone2_fd; /**< File descriptor for the port */

const uint16_t mot_pwm_min = 0x00;
const uint16_t mot_pwm_max = 0x1ff;

#define ARDRONE_GPIO_PORT       0x32524

#define ARDRONE_GPIO_PIN_MOTOR1     171
#define ARDRONE_GPIO_PIN_MOTOR2     172
#define ARDRONE_GPIO_PIN_MOTOR3     173
#define ARDRONE_GPIO_PIN_MOTOR4     174

#define ARDRONE_GPIO_PIN_IRQ_FLIPFLOP 175
#define ARDRONE_GPIO_PIN_IRQ_INPUT    176

uint32_t led_hw_values;
volatile bool motors_armed = false;

void actuators_ardrone_motor_status(void);

static inline void actuators_ardrone_reset_flipflop(void)
{
	gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_FLIPFLOP);
	gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_FLIPFLOP);
	sleep(2);
	//	int32_t stop = sys_time.nb_sec + 2;
	//	while (sys_time.nb_sec < stop);
	gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_FLIPFLOP);
}

static void *motor_update(void *data __attribute__((unused)))
{
	double t0 = timeNow_us();
	int8_t check_status = 100;
	printf("Motor control thread started!\r\n");
	motors_armed = true;
	while(1) {
		// 200Hz update interval
		usleep(5000);

		pthread_mutex_lock(&mot_mutex);
		actuators_ardrone_set_pwm(motor.pwm[0], motor.pwm[1], motor.pwm[2],	motor.pwm[3]);
		actuators_ardrone_set_leds(motor.led[0], motor.led[1], motor.led[2], motor.led[3]);
		pthread_mutex_unlock(&mot_mutex);

		// check motor status every 100th sample (2Hz)
		if (check_status-- <= 0){
			actuators_ardrone_motor_status();
			check_status = 10;
		}
	}
	motors_armed = false;
	return NULL;
}

bool actuators_ardrone_init(void)
{
	int i;
	led_hw_values = 0;
	for (i = 0; i < 4; i++){
		motor.pwm[i] = 0;
		motor.led[i] = MOT_LEDGREEN;
		motor.power[i] = 0.0f;
	}

	//open mot port
	actuator_ardrone2_fd = open("/dev/ttyO0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (actuator_ardrone2_fd == -1) {
		perror("open_port: Unable to open /dev/ttyO0 - ");
		return false;
	}
	fcntl(actuator_ardrone2_fd, F_SETFL, 0); //read calls are non blocking
	fcntl(actuator_ardrone2_fd, F_GETFL, 0);

	//set port options
	struct termios options;
	//Get the current options for the port
	tcgetattr(actuator_ardrone2_fd, &options);
	//Set the baud rates to 115200
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);

	options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
	options.c_iflag = 0; //clear input options
	options.c_lflag = 0; //clear local options
	options.c_oflag &= ~OPOST; //clear output options (raw output)

	//Set the new options for the port
	tcsetattr(actuator_ardrone2_fd, TCSANOW, &options);

	//reset IRQ flipflop - on error 106 read 1, this code resets 106 to 0
	gpio_setup_input(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_INPUT);
	actuators_ardrone_reset_flipflop();

	//all select lines active
	gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR1);
	gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR2);
	gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR3);
	gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR4);
	gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR1);
	gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR2);
	gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR3);
	gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR4);

	//configure motors
	uint8_t reply[256];
	int m;
	for (m = 0; m < 4; m++) {
		gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR1 + m);
		actuators_ardrone_cmd(0xe0, reply, 2);
		if (reply[0] != 0xe0 || reply[1] != 0x00) {
			printf("motor%d cmd=0x%02x reply=0x%02x\r\n", m + 1, (int)reply[0], (int)reply[1]);
		}
		actuators_ardrone_cmd(m + 1, reply, 1);
		gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR1 + m);
	}

	//all select lines active
	gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR1);
	gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR2);
	gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR3);
	gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_MOTOR4);

	//start multicast
	actuators_ardrone_cmd(0xa0, reply, 1);
	actuators_ardrone_cmd(0xa0, reply, 1);
	actuators_ardrone_cmd(0xa0, reply, 1);
	actuators_ardrone_cmd(0xa0, reply, 1);
	actuators_ardrone_cmd(0xa0, reply, 1);

	//reset IRQ flipflop - on error 176 reads 1, this code resets 176 to 0
	gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_FLIPFLOP);
	gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_FLIPFLOP);

	// Left Red, Right Green
	actuators_ardrone_set_leds(MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN, MOT_LEDGREEN);

	pthread_mutex_init(&mot_mutex, NULL);
	//start mot thread
	int rc = pthread_create(&mot_thread, NULL, motor_update, NULL);
	if (rc) {
		printf("[motor]: Return code from pthread_create(mot_thread) is %d\n", rc);
		return false;
	}
	return true;
}

int actuators_ardrone_cmd(uint8_t cmd, uint8_t *reply, int replylen)
{
	if (full_write(actuator_ardrone2_fd, &cmd, 1) < 0) {
		perror("actuators_ardrone_cmd: write failed");
		return -1;
	}
	return full_read(actuator_ardrone2_fd, reply, replylen);
}


void actuators_ardrone_motor_status(void)
{
	// Reset Flipflop sequence
	static uint8_t reset_flipflop_counter = 0;

	if (reset_flipflop_counter > 0) {
		reset_flipflop_counter--;

		if (reset_flipflop_counter == 10) {
			// Reset flipflop
			gpio_setup_output(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_FLIPFLOP);
			gpio_clear(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_FLIPFLOP);
		} else if (reset_flipflop_counter == 1) {
			// Listen to IRQ again
			gpio_set(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_FLIPFLOP);
		}
		return;
	}

	// If a motor IRQ line is set
	if (gpio_get(ARDRONE_GPIO_PORT, ARDRONE_GPIO_PIN_IRQ_INPUT) == 1) {
		if (motors_armed) {
			reset_flipflop_counter = 20;
		}
	}
}


#define BIT_NUMBER(VAL,BIT) (((VAL)>>BIT)&0x03)

void actuators_ardrone_led_run(void)
{
	static uint32_t previous_led_hw_values = 0x00;
	if (previous_led_hw_values != led_hw_values) {
		previous_led_hw_values = led_hw_values;
		actuators_ardrone_set_leds(BIT_NUMBER(led_hw_values, 0), BIT_NUMBER(led_hw_values, 2), BIT_NUMBER(led_hw_values, 4),
				BIT_NUMBER(led_hw_values, 6));
	}
}

void actuators_ardrone_set_power(float mot1, float mot2, float mot3, float mot4){
	int i;

	pthread_mutex_lock(&mot_mutex);
	motor.power[0] = mot1;
	motor.power[1] = mot2;
	motor.power[2] = mot3;
	motor.power[3] = mot4;

	for(i = 0; i < 4; i++) {
		// sat limits
		if(motor.power[i] < 0.0) motor.power[i] = 0.0;
		if(motor.power[i] > 1.0) motor.power[i] = 1.0;
		// scale
		motor.pwm[i] = mot_pwm_min + (uint16_t)(motor.power[i]*(mot_pwm_max-mot_pwm_min));
		// ensure sat not violated
		if(motor.pwm[i] < mot_pwm_min) motor.pwm[i] = mot_pwm_min;
		if(motor.pwm[i] > mot_pwm_max) motor.pwm[i] = mot_pwm_max;
	}
	pthread_mutex_unlock(&mot_mutex);
}

void actuators_ardrone_commit_color(uint8_t LED1, uint8_t LED2, uint8_t LED3, uint8_t LED4){
	pthread_mutex_lock(&mot_mutex);
	motor.led[0] = LED1;
	motor.led[1] = LED2;
	motor.led[2] = LED3;
	motor.led[3] = LED4;
	pthread_mutex_unlock(&mot_mutex);
}

void actuators_stop(void){
	pthread_mutex_lock(&mot_mutex);
	motor.power[0] = 0.0;
	motor.power[1] = 0.0;
	motor.power[2] = 0.0;
	motor.power[3] = 0.0;
	int i;
	for(i = 0; i < 4; i++) {
		if(motor.power[i] < 0.0) motor.power[i] = 0.0;
		if(motor.power[i] > 1.0) motor.power[i] = 1.0;
		motor.pwm[i] = 0;
	}
	actuators_ardrone_set_pwm(motor.pwm[0], motor.pwm[1], motor.pwm[2],	motor.pwm[3]);
	pthread_mutex_unlock(&mot_mutex);
}

/**
 * Write motor speed command
 * cmd = 001aaaaa aaaabbbb bbbbbccc ccccccdd ddddddd0
 */
void actuators_ardrone_set_pwm(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3)
{
	uint8_t cmd[5];
	cmd[0] = 0x20 | ((pwm0 & 0x1ff) >> 4);
	cmd[1] = ((pwm0 & 0x1ff) << 4) | ((pwm1 & 0x1ff) >> 5);
	cmd[2] = ((pwm1 & 0x1ff) << 3) | ((pwm2 & 0x1ff) >> 6);
	cmd[3] = ((pwm2 & 0x1ff) << 2) | ((pwm3 & 0x1ff) >> 7);
	cmd[4] = ((pwm3 & 0x1ff) << 1);
	full_write(actuator_ardrone2_fd, cmd, 5);
}

/**
 * Write LED command
 * cmd = 011rrrr0 000gggg0 (this is ardrone1 format, we need ardrone2 format)
 *
 *
 *  led0 = RearLeft
 *  led1 = RearRight
 *  led2 = FrontRight
 *  led3 = FrontLeft
 */

void actuators_ardrone_set_leds(uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3)
{
	uint8_t cmd[2];

	led0 &= 0x03;
	led1 &= 0x03;
	led2 &= 0x03;
	led3 &= 0x03;

	//printf("LEDS: %d %d %d %d \n", led0, led1, led2, led3);

	cmd[0] = 0x60 | ((led0 & 1) << 4) | ((led1 & 1) << 3) | ((led2 & 1) << 2) | ((led3 & 1) << 1);
	cmd[1] = ((led0 & 2) << 3) | ((led1 & 2) << 2) | ((led2 & 2) << 1) | ((led3 & 2) << 0);

	full_write(actuator_ardrone2_fd, cmd, 2);
}

void actuators_ardrone_close(void)
{
	if (actuator_ardrone2_fd != NULL)
		close(actuator_ardrone2_fd);
}
