#include "CciProtocol.h"
#include "usart.h"

uint8_t this_node_address = 0;

void SendShortMessage(uint8_t dst, uint8_t src, uint8_t id, uint8_t msgType, uint8_t * data, uint16_t len);
void SendLongMessage(uint8_t dst, uint8_t src, uint8_t id, uint8_t msgType, uint8_t * data, uint16_t len);

void init_cci(uint8_t node){
	this_node_address = node;
}

void SendMessageCan(uint8_t dst, uint8_t src, uint8_t id, uint8_t msgType, uint8_t * data, uint16_t len){
    if (len <= 8)
        SendShortMessage(dst, src, id, msgType, data, len);
    else
        SendLongMessage(dst, src, id, msgType, data, len);
}

// This function sends payload of 8 bytes only
void SendShortMessage(uint8_t dst, uint8_t src, uint8_t id, uint8_t msgType, uint8_t * data, uint16_t len){
    CCI_Message CciMsg = {0};
	uint32_t ID = CAN_ID_DEST(dst)
				| CAN_ID_SOURCE(src)
				| CAN_ID_MSGID(id)
				| CAN_ID_MSGTYPE(msgType)
				| CAN_ID_FRAMETYPE(CCI_FRAME_SHORT)
				| BIT(31);

    CciMsg.ID = ID;
    memcpy(&CciMsg.Data, data, len);
    CciMsg.Length = len;
    USART_send(USART2, (uint8_t *)&CciMsg, sizeof(CCI_Message));
}

void SendLongMessage(uint8_t dst, uint8_t src, uint8_t id, uint8_t msgType, uint8_t * data, uint16_t len){
    CCI_Message cciMsgLong[9] = {0};
	uint8_t msgCount = 0;
    uint32_t ID = 0;

    // start frame
    ID = CAN_ID_DEST(dst)
        | CAN_ID_SOURCE(src)
		| CAN_ID_MSGID(id)
		| CAN_ID_MSGTYPE(msgType)
		| CAN_ID_FRAMETYPE(CCI_FRAME_LONG_CTRL)
		| BIT(31);
    cciMsgLong[msgCount].ID = ID;
    cciMsgLong[msgCount].Length = 4;
    uint8_t flags = 0;
   	cciMsgLong[msgCount].Data[0] = ((flags & 0x0F) << 4) | (CCI_LONG_START & 0x0F);
	cciMsgLong[msgCount].Data[1] = (len >> 16) & 0xFF;
	cciMsgLong[msgCount].Data[2] = (len >> 8) & 0xFF;
	cciMsgLong[msgCount].Data[3] = (len) & 0xFF;
    msgCount++;

    uint8_t remainingSize = len;
    uint8_t * payload = data;

	do {
        ID = CAN_ID_DEST(dst)
				| CAN_ID_SOURCE(src)
				| CAN_ID_MSGID(id)
				| CAN_ID_MSGTYPE(msgType)
				| CAN_ID_FRAMETYPE(CCI_FRAME_LONG_DATA)
				| BIT(31);
        cciMsgLong[msgCount].ID = ID;
        if (remainingSize == 0) {
    		cciMsgLong[msgCount].Length = 0;
    		break;
    	}
        uint8_t length = (uint8_t)(remainingSize < 8 ? remainingSize : 8);
        memcpy(&cciMsgLong[msgCount].Data[0], payload, length);
        cciMsgLong[msgCount].Length = length;

		uint8_t messageLength = cciMsgLong[msgCount].Length;
		remainingSize -= messageLength;
		payload += messageLength;
		++msgCount;
	} while (remainingSize > 0);

    int i;
    for (i = 0; i < msgCount; i++){
    	USART_send(USART2, (uint8_t *)&cciMsgLong[i], sizeof(CCI_Message));
    }
}

bool decodeCciHeader(uint8_t * rxdata, CCI_MessageFull * cciMsg){
    uint32_t ID = 0;

    uint8_t id_tmp[4] = {0};
    memcpy(&id_tmp[0], rxdata, 4);
    // change from LSByte to MSByte
    ID |= id_tmp[0];
    ID |= id_tmp[1] << 8;
    ID |= id_tmp[2] << 16;
    ID |= id_tmp[3] << 24;

    cciMsg->src = (ID >> CAN_ID_SOURCE_shift) & CAN_ID_SOURCE_mask;
    cciMsg->dst = (ID >> CAN_ID_DEST_shift) & CAN_ID_DEST_mask;
    cciMsg->msgType = (ID >> CAN_ID_MSGTYPE_shift) & CAN_ID_MSGTYPE_mask;
    cciMsg->msgID = (ID >> CAN_ID_MSGID_shift) & CAN_ID_MSGID_mask;
    cciMsg->frame = (ID >> CAN_ID_FRAMETYPE_shift) & CAN_ID_FRAMETYPE_mask;
    cciMsg->message_complete = false;

    // ignore all own messages
    if (cciMsg->src == this_node_address)
        return false;
    // if the message is not intended for this node or broadcast, ignore!
    if (!((cciMsg->dst == this_node_address) || (cciMsg->dst == 255)))
        return false;

    uint16_t unit_length = 0;
    uint8_t write_loc = 0;
    unit_length |= *(rxdata + 4);
    unit_length |= *(rxdata + 5) << 8;

    switch (cciMsg->frame){
        case CCI_FRAME_SHORT:
            cciMsg->length = unit_length;
            cciMsg->remaining_bytes = 0;
            memcpy(&cciMsg->data[0], rxdata + 8, unit_length);
            break;
        case CCI_FRAME_LONG_CTRL:
            // setup to receive subsequent frames
            cciMsg->length = 0;
            cciMsg->length |= *(rxdata + 9) << 16;
            cciMsg->length |= *(rxdata + 10) << 8;
            cciMsg->length |= *(rxdata + 11);
            cciMsg->remaining_bytes = cciMsg->length;
            // clear buffer
            memset(&cciMsg->data[0], 0, 64);
            break;
        case CCI_FRAME_LONG_DATA:
            // append data
            write_loc = cciMsg->length - cciMsg->remaining_bytes;
            memcpy(&cciMsg->data[write_loc], rxdata + 8, unit_length);
            cciMsg->data[write_loc + unit_length] = 0;
            cciMsg->remaining_bytes -= unit_length;
            break;
    }
    if (!cciMsg->remaining_bytes){
        cciMsg->data[cciMsg->length] = '\0';
        cciMsg->message_complete = true;
    }

    return cciMsg->message_complete;
}
