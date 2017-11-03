/*
 * l3g4200d.c
 *
 *  Created on: 03 Nov 2017
 *      Author: yreddi
 */

#include "l3g4200d.h"

#define GYRO_ADDRESS 0b1101000

char read_register(I2C_TypeDef* I2Cx, char reg);

void init_gyro(I2C_TypeDef* I2Cx){
	char config[2] = {0};

	// power up
	config[0] = 0x20;
	config[1] = 0x0F;
	I2C_start(I2Cx, GYRO_ADDRESS << 1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2Cx, config[0]);
	I2C_write(I2Cx, config[1]);
	I2C_stop(I2Cx);

	// set full scale range
	config[0] = 0x23;
	config[1] = 0x30;
	I2C_start(I2Cx, GYRO_ADDRESS << 1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2Cx, config[0]);
	I2C_write(I2Cx, config[1]);
	I2C_stop(I2Cx);
}

void read_gyro(I2C_TypeDef* I2Cx, int16_t gyro[3]){
	int i;
	char rx_data[6] = {0};
	// read data
/*	I2C_start(I2Cx, GYRO_ADDRESS << 1, I2C_Direction_Transmitter); 	// start a transmission in Master transmitter mode
	I2C_write(I2Cx, 0x28);
	I2C_stop(I2Cx);

	I2C_start(I2Cx, GYRO_ADDRESS << 1, I2C_Direction_Receiver); 	// start a transmission in Master receiver mode
	rx_data[0] = I2C_read_ack(I2Cx);
	rx_data[5] = I2C_read_nack(I2Cx);

	I2C_stop(I2Cx);
*/
	for (i = 0; i < 6; i++){
		rx_data[i] = read_register(I2C1, 0x28 + i);
		Delay(100000);
	}

	int16_t gyro_int[3] = {0};
	gyro[0] = (int16_t)(((uint16_t)rx_data[1] << 8) | (uint16_t)rx_data[0]);
	gyro[1] = (int16_t)(((uint16_t)rx_data[3] << 8) | (uint16_t)rx_data[2]);
	gyro[2] = (int16_t)(((uint16_t)rx_data[5] << 8) | (uint16_t)rx_data[4]);
}

char read_register(I2C_TypeDef* I2Cx, char reg){
	char tmp;
	I2C_start(I2Cx, GYRO_ADDRESS << 1, I2C_Direction_Transmitter); 	// start a transmission in Master transmitter mode
	I2C_write(I2Cx, reg);
	I2C_stop(I2Cx);

	I2C_start(I2Cx, GYRO_ADDRESS << 1, I2C_Direction_Receiver); 	// start a transmission in Master receiver mode
	tmp = I2C_read_nack(I2Cx);
	I2C_stop(I2Cx);
	return tmp;
}
