/*
 * l3g4200d.h
 *
 *  Created on: 03 Nov 2017
 *      Author: yreddi
 */

#ifndef L3G4200D_H_
#define L3G4200D_H_

#include "i2c.h"

void init_gyro(I2C_TypeDef* I2Cx);
void read_gyro(I2C_TypeDef* I2Cx, int16_t gyro[3]);

#endif /* L3G4200D_H_ */
