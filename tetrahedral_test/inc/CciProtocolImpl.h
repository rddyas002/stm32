#ifndef _CCIPROTOCOLIMPL_H_
#define _CCIPROTOCOLIMPL_H_

#if !defined(CAN_PROTOCOL_VERSION) || CAN_PROTOCOL_VERSION != 2
#error Missing or unknown CAN protocol version
#endif

#define CAN_ID_SOURCE_shift		8
#define CAN_ID_SOURCE_mask		0xff
#define CAN_ID_SOURCE(x) (((x) & CAN_ID_SOURCE_mask) << CAN_ID_SOURCE_shift)
#define CCI_MSG_SOURCE(m) (((m)->ID >> CAN_ID_SOURCE_shift) & CAN_ID_SOURCE_mask)

#define CAN_ID_DEST_shift		16
#define CAN_ID_DEST_mask		0xff
#define CAN_ID_DEST(x) (((x) & CAN_ID_DEST_mask) << CAN_ID_DEST_shift)
#define CCI_MSG_DESTINATION(m) (((m)->ID >> CAN_ID_DEST_shift) & CAN_ID_DEST_mask)

#define CAN_ID_MSGID_shift		0
#define CAN_ID_MSGID_mask		0xff
#define CAN_ID_MSGID(x) (((x) & CAN_ID_MSGID_mask) << CAN_ID_MSGID_shift)
#define CCI_MSG_MSGID(m) (((m)->ID >> CAN_ID_MSGID_shift) & CAN_ID_MSGID_mask)

#define CAN_ID_MSGTYPE_shift	26
#define CAN_ID_MSGTYPE_mask		0x07
#define CAN_ID_MSGTYPE(x) (((x) & CAN_ID_MSGTYPE_mask) << CAN_ID_MSGTYPE_shift)
#define CCI_MSG_MSGTYPE(m) (((m)->ID >> CAN_ID_MSGTYPE_shift) & CAN_ID_MSGTYPE_mask)

#define CAN_ID_FRAMETYPE_shift	24
#define CAN_ID_FRAMETYPE_mask	0x03 
#define CAN_ID_FRAMETYPE(x) (((x) & CAN_ID_FRAMETYPE_mask) << CAN_ID_FRAMETYPE_shift)
#define CCI_MSG_FRAMETYPE(m) (((m)->ID >> CAN_ID_FRAMETYPE_shift) & CAN_ID_FRAMETYPE_mask)

// CanNodeEmulator.cs line189 ->
#define TC_RET_OK					0;
#define TC_RET_EXECUTION_DELAYED	1;
#define TC_RET_NOT_IMPLEMENTED		255;

/* ************************************************************************** */

/* New Message types */
#define CCI_MESSAGETYPE_TIMESYNC		0x00 //00000
#define CCI_MESSAGETYPE_EVENT			0x01 //00001
#define CCI_MESSAGETYPE_TCMREQ			0x02 //00010
#define CCI_MESSAGETYPE_TCMRESP			0x03 //00011
#define CCI_MESSAGETYPE_TLMREQ			0x04 //00100
#define CCI_MESSAGETYPE_TLMRESP			0x05 //00101
#define CCI_MESSAGETYPE_BULK			0x06 //00110
#define CCI_MESSAGETYPE_RESERVED		0x07

/* ************************************************************************** */

/* Long Frame types */
#define CCI_FRAME_SHORT			        0x00 // 0b00
#define CCI_FRAME_LONG_CTRL             0x02 // 0b10
#define CCI_FRAME_LONG_DATA             0x03 // 0b11

// CanTransport_EOSAT1.cs:422
/* Long Frame Packet types */
#define CCI_LONG_START				    0x00 // 0000
#define CCI_LONG_ACK				    0x01 // 0001
#define CCI_LONG_NACK				    0x02 // 0010 
#define CCI_LONG_DONE				    0x03 // 0011

//#define CCI_BULK_FID_DATA				0x10 

/* ************************************************************************** */

#define CCI_ASYNC_RESULT_INIT      (1<<0)
#define CCI_ASYNC_RESULT_AVAILABLE (1<<1)
#define CCI_ASYNC_RESULT_SUCCESS   (1<<2)

/**
* This macro check if the asyncResult is available.  If false, the asyncResult
* is still waiting for a reply message or a deadline to expire.
*/
#define CCI_IS_ASYNC_RESULT_AVAILABLE(asyncResult) \
    (((asyncResult)->Flags & (CCI_ASYNC_RESULT_AVAILABLE)) \
        == (CCI_ASYNC_RESULT_AVAILABLE))

/**
* This macro check if the asyncResult has succeded.  The CAN message has
* been updated and is valid. The application can use the message.
* The Error field is undefined.
*/
#define CCI_HAS_ASYNC_RESULT_SUCCEEDED(asyncResult) \
    (((asyncResult)->Flags & (CCI_ASYNC_RESULT_AVAILABLE|CCI_ASYNC_RESULT_SUCCESS)) \
        == (CCI_ASYNC_RESULT_AVAILABLE|CCI_ASYNC_RESULT_SUCCESS))

/**
* This macro check if the asyncResult as failed.  The @Error field is
* valid and can be used to determine what error occured.  Use the
* CCI_GetErrorMessage() function to get a user printable string
* of the error.
*/
#define CCI_HAS_ASYNC_RESULT_FAILED(asyncResult) \
    (((asyncResult)->Flags & (CCI_ASYNC_RESULT_AVAILABLE|CCI_ASYNC_RESULT_SUCCESS)) \
        == (CCI_ASYNC_RESULT_AVAILABLE))

/**
* This macro check if the asyncResult has timedout. This will happen when
* the deadline has been reached and no reply message has been received.
*/
#define CCI_HAS_ASYNC_RESULT_TIMEDOUT(asyncResult) \
    ((((asyncResult)->Flags & (CCI_ASYNC_RESULT_AVAILABLE|CCI_ASYNC_RESULT_SUCCESS)) \
        == (CCI_ASYNC_RESULT_AVAILABLE)) && ((asyncResult)->Error == ETIMEDOUT))

/* ************************************************************************** */

#endif //_CCIPROTOCOLIMPL_H_

