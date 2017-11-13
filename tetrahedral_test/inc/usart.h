/*
 * usart.h
 *
 *  Created on: 04 Nov 2017
 *      Author: RDDYA
 */

#ifndef USART_H_
#define USART_H_

#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_usart.h>

void init_USART1(uint32_t baudrate);
void init_USART2(uint32_t baudrate);
void USART_puts(USART_TypeDef* USARTx, volatile char *s);
void USART_send(USART_TypeDef* USARTx, uint8_t *s, uint8_t length);
void USART_sendInt(uint8_t * data, uint8_t length);

#endif /* USART_H_ */
