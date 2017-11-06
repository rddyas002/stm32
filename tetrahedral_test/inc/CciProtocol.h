/*
 * CciProtocol.h
 *
 *  Created on: 06 Nov 2017
 *      Author: yreddi
 */

#ifndef CCIPROTOCOL_H_
#define CCIPROTOCOL_H_

#define CAN_PROTOCOL_VERSION 2
#include "CciProtocolImpl.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define CCI_MAX_PAYLOAD_LENGTH 8
/** The maximum payload lenght of a RAW Message */
#define RAW_MAX_PAYLOAD_LENGTH 100

# define CCI_ATTR_PACKED     __attribute__((packed))
# define CCI_ATTR_ALIGNED(n) __attribute__((aligned(n)))

typedef struct{
	uint32_t ID;					/* Full 32-bit Message ID */
	uint32_t Length : 16;			/* Data length field */
	uint32_t Flags : 8;				/* Control flags field */
    uint32_t Reserved:8;			/* Reserved for asyncResult use */
    uint8_t Data[CCI_MAX_PAYLOAD_LENGTH] CCI_ATTR_ALIGNED(8);
}CCI_ATTR_PACKED CCI_Message;

typedef struct{
    uint8_t src;
    uint8_t dst;
    uint8_t msgType;
    uint8_t msgID;
    uint8_t frame;
    uint16_t length;
    uint16_t remaining_bytes;
    bool message_complete;
    uint8_t data[64];
}CCI_ATTR_PACKED CCI_MessageFull;

void init_cci(uint8_t node);
bool decodeCciHeader(uint8_t * rxdata, CCI_MessageFull * cciMsg);
void SendMessageCan(uint8_t dst, uint8_t src, uint8_t id, uint8_t msgType, uint8_t * data, uint16_t len);

#endif /* CCIPROTOCOL_H_ */
