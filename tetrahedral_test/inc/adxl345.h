/*
 * adxl345.h
 *
 *  Created on: 06 Nov 2017
 *      Author: yreddi
 */

#ifndef ADXL345_H_
#define ADXL345_H_

#include "i2c.h"

void init_accel(I2C_TypeDef* I2Cx);
void read_accel(I2C_TypeDef* I2Cx, int16_t gyro[3]);
void L3G4200D_I2C_BufferRead(I2C_TypeDef* I2Cx, u8 slAddr, u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);

#endif /* ADXL345_H_ */
