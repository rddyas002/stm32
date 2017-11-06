#include "hmc5883l.h"

void I2Cx_ByteWrite(I2C_TypeDef* I2Cx, u8 slaveAddr, u8* pBuffer, u8 WriteAddr);
void HMC5883L_SetMode(I2C_TypeDef* I2Cx, uint8_t newMode);
void HMC5883L_SetGain(I2C_TypeDef* I2Cx, uint8_t gain);
uint8_t HMC5883L_ReadByte(I2C_TypeDef* I2Cx, u8 slaveAddr, uint8_t address)

uint8_t HMC5883Lmode = 1;

void init_mag(I2C_TypeDef* I2Cx) {
	// write CONFIG_A register
	uint8_t tmp = (HMC5883L_AVERAGING_8
			<< (HMC5883L_CRA_AVERAGE_BIT - HMC5883L_CRA_AVERAGE_LENGTH + 1))
			| (HMC5883L_RATE_15
					<< (HMC5883L_CRA_RATE_BIT - HMC5883L_CRA_RATE_LENGTH + 1))
			| (HMC5883L_BIAS_NORMAL
					<< (HMC5883L_CRA_BIAS_BIT - HMC5883L_CRA_BIAS_LENGTH + 1));
	I2Cx_ByteWrite(I2C1, HMC5883L_DEFAULT_ADDRESS, &tmp, HMC5883L_RA_CONFIG_A);

	// write CONFIG_B register
	HMC5883L_SetGain(I2C1, HMC5883L_GAIN_1090);

	// write MODE register
	HMC5883L_SetMode(I2C1, HMC5883L_MODE_SINGLE);
}

void HMC5883L_SetGain(I2C_TypeDef* I2Cx, uint8_t gain) {
	// use this method to guarantee that bits 4-0 are set to zero, which is a
	// requirement specified in the datasheet;
	uint8_t tmp = gain << (HMC5883L_CRB_GAIN_BIT - HMC5883L_CRB_GAIN_LENGTH + 1);
	I2Cx_ByteWrite(I2Cx, HMC5883L_DEFAULT_ADDRESS, &tmp, HMC5883L_RA_CONFIG_B);
}

void HMC5883L_SetMode(I2C_TypeDef* I2Cx, uint8_t newMode) {
	// use this method to guarantee that bits 7-2 are set to zero, which is a
	// requirement specified in the datasheet;
	uint8_t tmp = HMC5883Lmode << (HMC5883L_MODEREG_BIT - HMC5883L_MODEREG_LENGTH + 1);
	I2Cx_ByteWrite(I2Cx, HMC5883L_DEFAULT_ADDRESS, &tmp, HMC5883L_RA_MODE);
	HMC5883Lmode = newMode; // track to tell if we have to clear bit 7 after a read
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
