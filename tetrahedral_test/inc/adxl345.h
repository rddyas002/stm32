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

#endif /* ADXL345_H_ */
