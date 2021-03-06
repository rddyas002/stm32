/*
 * l3g4200d.c
 *
 *  Created on: 03 Nov 2017
 *      Author: yreddi
 */

#include "l3g4200d.h"
#include <math.h>

#define M_PI_f 3.14159265358979323846f

#define GYRO_ADDRESS (unsigned char)0b1101000
#define GYRO_GAIN	(70E-3f)
//https://github.com/jmondi/EHF_FreeRTOS/tree/master/Libraries/ML_Lib/src

void init_gyro(I2C_TypeDef* I2Cx){
	// power up
	uint8_t ret = I2C_WriteByte(I2Cx, GYRO_ADDRESS << 1, 0x20, 0b00111111);

	// set full scale range
	ret = I2C_WriteByte(I2Cx, GYRO_ADDRESS << 1, 0x23, 0x30);
}

float read_gyro(I2C_TypeDef* I2Cx, float gyro_f[3], int8_t * temperature){
	int16_t gyro[3] = {0};
	uint8_t rx_data[8] = {0};

	I2C_ReadBytes(I2Cx, GYRO_ADDRESS << 1, 0x28|0x80, &rx_data[0], 8);
	*temperature = (int8_t)rx_data[0];

	gyro[0] = (int16_t)(((uint16_t)rx_data[7] << 8) | (uint16_t)rx_data[6]);
	gyro[1] = (int16_t)(((uint16_t)rx_data[3] << 8) | (uint16_t)rx_data[2]);
	gyro[2] = (int16_t)(((uint16_t)rx_data[5] << 8) | (uint16_t)rx_data[4]);

	gyro_f[0] = (float)gyro[0]*GYRO_GAIN;//*M_PI_f/180;
	gyro_f[1] = (float)gyro[1]*GYRO_GAIN;//*M_PI_f/180;
	gyro_f[2] = (float)gyro[2]*GYRO_GAIN;//*M_PI_f/180;

	float w_mag = sqrt(gyro_f[0]*gyro_f[0] + gyro_f[1]*gyro_f[1] + gyro_f[2]*gyro_f[2]);
	return w_mag;
}
