#include "adxl345.h"

#define ACCEL_ADDRESS	0x53

void init_accel(I2C_TypeDef* I2Cx){
	char config[2] = {0};

	// power up
	config[0] = 0x2D;
	config[1] = 0x08;
	I2C_start(I2Cx, ACCEL_ADDRESS << 1, I2C_Direction_Transmitter);
	I2C_write(I2Cx, config[0]);
	I2C_write(I2Cx, config[1]);
	I2C_stop(I2Cx);

	// set resolution 3.9mg/LSB and range 16g
	config[0] = 0x31;
	config[1] = 0x0B;
	I2C_start(I2Cx, ACCEL_ADDRESS << 1, I2C_Direction_Transmitter);
	I2C_write(I2Cx, config[0]);
	I2C_write(I2Cx, config[1]);
	I2C_stop(I2Cx);
}

void read_accel(I2C_TypeDef* I2Cx, int16_t accel[3]){
	int i;
	char rx_data[6] = {0};

	for (i = 0; i < 6; i++){
		rx_data[i] = read_register(I2C1, ACCEL_ADDRESS, 0x32 + i);
		Delay(100000);
	}

	accel[0] = (int16_t)(((uint16_t)rx_data[1] << 8) | (uint16_t)rx_data[0]);
	accel[1] = (int16_t)(((uint16_t)rx_data[3] << 8) | (uint16_t)rx_data[2]);
	accel[2] = (int16_t)(((uint16_t)rx_data[5] << 8) | (uint16_t)rx_data[4]);
}
