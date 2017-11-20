/*
 * motors.h
 *
 *  Created on: 18 Nov 2017
 *      Author: RDDYA
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef ACTUATORS_ARDRONE_NB
#define ACTUATORS_ARDRONE_NB 4
#endif

#define SERVOS_TICS_OF_USEC(_v) (_v)

#define MOT_LEDOFF 0
#define MOT_LEDRED 1
#define MOT_LEDGREEN 2
#define MOT_LEDORANGE 3

bool actuators_ardrone_init(void);
void actuators_ardrone_set_power(float mot1, float mot2, float mot3, float mot4);
int actuators_ardrone_cmd(uint8_t cmd, uint8_t *reply, int replylen);
void actuators_ardrone_set_pwm(uint16_t pwm0, uint16_t pwm1, uint16_t pwm2, uint16_t pwm3);
void actuators_ardrone_set_leds(uint8_t led0, uint8_t led1, uint8_t led2, uint8_t led3);
void actuators_stop(void);
void actuators_ardrone_close(void);


#endif /* MOTORS_H_ */
