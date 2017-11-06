#include <string.h>
#include <stdlib.h>

#include "usart.h"
#include "CciProtocol.h"

#define MAX_STRLEN 12
#define MAX_CCI_LEN 16
volatile char received_string[MAX_STRLEN+1]; 	 // this will hold the recieved string
volatile char cci_rx[MAX_CCI_LEN]; 	 // this will hold the recieved string
CCI_MessageFull * cciRxMsg;

void init_USART1(uint32_t baudrate){
	GPIO_InitTypeDef GPIO_InitStruct; 		// this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; 	// this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; 	// this is used to configure the NVIC (nested vector interrupt controller)

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// configure gpio
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; 	// Pins 9 (TX) and 10 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 				// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	// configure pin alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// configure usart
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	// Enable Rx interrupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 	// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 	// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 	// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

void init_USART2(uint32_t baudrate){
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// configure gpio
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; 	// Pins 2 (TX) and 3 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 				// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;			// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;				// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;				// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOA, &GPIO_InitStruct);						// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	// configure pin alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	// configure usart
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART2, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting

	// Enable Rx interrupt
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;		 	// we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 	// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 	// the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART2 peripheral
	USART_Cmd(USART2, ENABLE);

	cciRxMsg = (CCI_MessageFull * ) malloc(sizeof(CCI_MessageFull));
}

void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s++);
	}
}

void USART_send(USART_TypeDef* USARTx, uint8_t *s, uint8_t length){
	uint8_t count = 0;
	while(count++ != length){
		// wait until data register is empty
		while(!(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s++);
	}
}

void USART1_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
/*
		char t;
		// clear received buffer
		while(USART_GetStatus(USART_FLAG_RXNE)){
			t = USART1->DR;
		}
*/
		static uint8_t cnt = 0; 	// this counter is used to determine the string length
		char t = USART1->DR; 		// the character from the USART1 data register is saved in t
		//while( !(USART1->SR & 0x00000040) );
		//USART_SendData(USART1, t);
		//GPIO_ToggleBits(GPIOD, GPIO_Pin_12);

		if(cnt < MAX_CCI_LEN){
			received_string[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			received_string[cnt] = '\0';
			USART_puts(USART1, received_string);
			cnt = 0;
		}
	}
}

void USART2_IRQHandler(void){
	// check if the USART2 receive interrupt flag was set
	if( USART_GetITStatus(USART2, USART_IT_RXNE) ){
		static uint8_t cnt = 0;
		char t = USART2->DR;

		if(cnt < MAX_CCI_LEN){
			cci_rx[cnt] = t;
			cnt++;
		}
		else{ // otherwise reset the character counter and print the received string
			cci_rx[cnt] = t;
			decodeCciHeader(&cci_rx[0], cciRxMsg);
			//USART_puts(USART2, received_string);
			//GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
			cnt = 0;
		}
	}
}
