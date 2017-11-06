/*
 * i2c.h
 *
 *  Created on: 03 Nov 2017
 *      Author: yreddi
 */

#ifndef I2C_H_
#define I2C_H_

#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>

void init_I2C1(void);
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
void I2C_stop(I2C_TypeDef* I2Cx);
void Delay(__IO uint32_t nCount);
char read_register(I2C_TypeDef* I2Cx, char address, char reg);

#endif /* I2C_H_ */
