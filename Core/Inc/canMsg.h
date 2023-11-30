/*
 * canMsg.h
 *
 *  Created on: Nov 25, 2022
 *      Author: PC
 */

#ifndef INC_CANMSG_H_
#define INC_CANMSG_H_

#include "stdint.h"

#define MAX_CAN_MSG_DATA_COUNT          8
#define MAX_STD_FILTER_NUM              0
#define MAX_EXT_FILTER_NUM              0
/***ecr*******************************************/
#define CAN_MSG_DATA_SENDER_ADDR_INDEX  0
#define CAN_MSG_DATA_TIMESTAMP_INDEX    2
#define MAX_CAN_MSG_DATA_COUNT               8
#define CAN_MSG_DATA_SENDER_ADDR_INDEX       0
#define CAN_MSG_DATA_TIMESTAMP_INDEX         2
#define CAN_MSG_DATA_PAYLOAD_0_INDEX         3
#define CAN_MSG_DATA_PAYLOAD_1_INDEX         4
#define CAN_MSG_DATA_PAYLOAD_2_INDEX         5
#define CAN_MSG_DATA_PAYLOAD_3_INDEX         6
#define CAN_MSG_DATA_PAYLOAD_4_INDEX         7
/*************************************************/



#define CAN_OK              (0)
#define CAN_FAILINIT        (1)
#define CAN_FAILTX          (2)
#define CAN_MSGAVAIL        (3)
#define CAN_NOMSG           (4)
#define CAN_CTRLERROR       (5)
#define CAN_GETTXBFTIMEOUT  (6)
#define CAN_SENDMSGTIMEOUT  (7)
#define CAN_FAIL            (0xff)



typedef struct CanMsg__Struct
{
	uint32_t Identifier;          /*!< Specifies the identifier.
                                     This parameter must be a number between:
                                      - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
                                      - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID                  */

	uint32_t IdType;              /*!< Specifies the identifier type for the message that will be
                                       transmitted.
                                       This parameter can be a value of @ref FDCAN_id_type                */

	uint32_t FrameType;           /*!< Specifies the frame type of the message that will be transmitted.
                                     This parameter can be a value of @ref FDCAN_frame_type               */


	uint32_t DataLength;          /*!< Specifies the length of the frame that will be transmitted.
	                                        This parameter can be a value of @ref FDCAN_data_length_code  */


	uint8_t Payload[MAX_CAN_MSG_DATA_COUNT];
}torkCanMsg;

#endif /* INC_CANMSG_H_ */
