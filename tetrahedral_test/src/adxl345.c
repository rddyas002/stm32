#include "adxl345.h"

#define ACCEL_ADDRESS	(unsigned char)0x53

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

void L3G4200D_I2C_BufferRead(I2C_TypeDef* I2Cx, u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead)
{
  /* While the bus is busy */
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send L3G4200D_Magn address for write */
  I2C_Send7bitAddress(I2Cx, slAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Clear EV6 by setting again the PE bit */
  I2C_Cmd(I2Cx, ENABLE);

  /* Send the L3G4200D_Magn's internal address to write to */
  I2C_SendData(I2Cx, ReadAddr);

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send STRAT condition a second time */
  I2C_GenerateSTART(I2Cx, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

  /* Send L3G4200D address for read */
  I2C_Send7bitAddress(I2Cx, slAddr, I2C_Direction_Receiver);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

  /* While there is data to be read */
  while(NumByteToRead)
  {
    if(NumByteToRead == 1)
    {
      /* Disable Acknowledgement */
      I2C_AcknowledgeConfig(I2Cx, DISABLE);

      /* Send STOP Condition */
      I2C_GenerateSTOP(I2Cx, ENABLE);
    }

    /* Test on EV7 and clear it */
    if(I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
    {
      /* Read a byte from the L3G4200D */
      *pBuffer = I2C_ReceiveData(I2Cx);

      /* Point to the next location where the byte read will be saved */
      pBuffer++;

      /* Decrement the read bytes counter */
      NumByteToRead--;
    }
  }

  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2Cx, ENABLE);
}
