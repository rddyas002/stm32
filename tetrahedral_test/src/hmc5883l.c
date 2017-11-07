#include "hmc5883l.h"

void HMC5883L_WriteByte(I2C_TypeDef* I2Cx, u8 slaveAddr, uint8_t address, uint8_t data);
uint8_t HMC5883L_ReadByte(I2C_TypeDef* I2Cx, u8 slaveAddr, uint8_t address);
void HMC5883L_ReadBytes(I2C_TypeDef* I2Cx, u8 slaveAddr, uint8_t startAddress, uint8_t * data, uint8_t bytes);

#define HMC5883L_GAIN 0.92f

void init_mag(I2C_TypeDef* I2Cx) {
	// write CONFIG_A register
	uint8_t tmp = 0b00011100;
	HMC5883L_WriteByte(I2Cx, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_A, tmp);

	// write CONFIG_B register
	tmp = 0b00100000;
	HMC5883L_WriteByte(I2Cx, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_CONFIG_B, tmp);

	// write MODE register
	tmp = 0b0;
	HMC5883L_WriteByte(I2Cx, HMC5883L_DEFAULT_ADDRESS, HMC5883L_RA_MODE, tmp);
}

void HMC5883L_WriteByte(I2C_TypeDef* I2Cx, u8 slaveAddr, uint8_t address, uint8_t data) {

	// Wait while I2C busy
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send start
	I2C_GenerateSTART(I2Cx, ENABLE);

	// Wait for EV5
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send HMC5883L address for write
	I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);

	// Wait for EV6
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// Send register address
	I2C_SendData(I2Cx, address);

	// Wait for EV8
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// Send new data
	I2C_SendData(I2Cx, data);

	// Wait for EV8
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// Generate stop
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

uint8_t HMC5883L_ReadByte(I2C_TypeDef* I2Cx, u8 slaveAddr, uint8_t address) {

	uint8_t ret;

	// Wait while I2C busy
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send start
	I2C_GenerateSTART(I2Cx, ENABLE);

	// Wait for EV5
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))	;

	// Send HMC5883L address for write
	I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Transmitter);

	// Wait for EV6
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// Send register address
	I2C_SendData(I2Cx, address);

	// Wait for EV8
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// Repeated start
	I2C_GenerateSTART(I2Cx, ENABLE);

	// Wait for EV5
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))	;

	// Send HMC5883L address for read
	I2C_Send7bitAddress(I2Cx, slaveAddr, I2C_Direction_Receiver);

	// Disable ACK
	I2C_AcknowledgeConfig(I2Cx, DISABLE);

	// Wait for EV6
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	// Generate stop
	I2C_GenerateSTOP(I2Cx, ENABLE);

	// Wait for EV7
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));

	ret = I2C_ReceiveData(I2Cx);

	// Enable ACK
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	return ret;
}

float read_mag(I2C_TypeDef* I2Cx, float B_f[3]) {
	uint8_t data[6] = {0};
	int16_t B[3] = {0};
	I2C_ReadBytes(I2C1, HMC5883L_DEFAULT_ADDRESS, 0x03, &data[0], 6);
	B[0] = data[1];
	B[0] |= data[0] << 8;
	B[1] = data[5];
	B[1] |= data[4] << 8;
	B[2] = data[3];
	B[2] |= data[2] << 8;

	B_f[0] = (float)B[0]*HMC5883L_GAIN;
	B_f[1] = (float)B[1]*HMC5883L_GAIN;
	B_f[2] = (float)B[2]*HMC5883L_GAIN;

	float B_mag = sqrt(B_f[0]*B_f[0] + B_f[1]*B_f[1] + B_f[2]*B_f[2]);

	B_f[0] /= B_mag;
	B_f[1] /= B_mag;
	B_f[2] /= B_mag;

	return B_mag;
}
