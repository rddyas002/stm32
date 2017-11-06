/*
 * l3g4200d.c
 *
 *  Created on: 03 Nov 2017
 *      Author: yreddi
 */

#include "l3g4200d.h"

#define GYRO_ADDRESS 0b1101000
//https://github.com/jmondi/EHF_FreeRTOS/tree/master/Libraries/ML_Lib/src

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

	for (i = 0; i < 6; i++){
		rx_data[i] = read_register(I2C1, GYRO_ADDRESS, 0x28 + i);
		Delay(100000);
	}


	gyro[0] = (int16_t)(((uint16_t)rx_data[1] << 8) | (uint16_t)rx_data[0]);
	gyro[1] = (int16_t)(((uint16_t)rx_data[3] << 8) | (uint16_t)rx_data[2]);
	gyro[2] = (int16_t)(((uint16_t)rx_data[5] << 8) | (uint16_t)rx_data[4]);
}
