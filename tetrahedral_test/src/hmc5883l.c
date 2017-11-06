#include "hmc5883l.h"

void HMC5883L_WriteByte(I2C_TypeDef* I2Cx, u8 slaveAddr, uint8_t address, uint8_t data);
uint8_t HMC5883L_ReadByte(I2C_TypeDef* I2Cx, u8 slaveAddr, uint8_t address);

void init_mag(I2C_TypeDef* I2Cx) {
	// write CONFIG_A register
	uint8_t tmp = 0b00011100;
	HMC5883L_WriteByte(I2C1, HMC5883L_DEFAULT_ADDRESS, tmp, HMC5883L_RA_CONFIG_A);

	// write CONFIG_B register
	tmp = 0b00100000;
	HMC5883L_WriteByte(I2Cx, HMC5883L_DEFAULT_ADDRESS, tmp, HMC5883L_RA_CONFIG_B);

	// write MODE register
	tmp = 0b0;
	HMC5883L_WriteByte(I2Cx, HMC5883L_DEFAULT_ADDRESS, tmp, HMC5883L_RA_MODE);
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

void read_mag(I2C_TypeDef* I2Cx, int16_t B[3]) {
	// Read x
	B[0] = HMC5883L_ReadByte(I2C1, HMC5883L_DEFAULT_ADDRESS, 0x03) << 8;
	B[0] |= HMC5883L_ReadByte(I2C1, HMC5883L_DEFAULT_ADDRESS, 0x04);

	// Read y
	B[1] = HMC5883L_ReadByte(I2C1, HMC5883L_DEFAULT_ADDRESS, 0x07) << 8;
	B[1] |= HMC5883L_ReadByte(I2C1, HMC5883L_DEFAULT_ADDRESS, 0x08);

	// Read z
	B[2] = HMC5883L_ReadByte(I2C1, HMC5883L_DEFAULT_ADDRESS, 0x05) << 8;
	B[2] |= HMC5883L_ReadByte(I2C1, HMC5883L_DEFAULT_ADDRESS, 0x06);

}
