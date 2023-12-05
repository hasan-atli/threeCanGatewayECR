/*
 * basicApp.h
 *
 *  Created on: Jun 26, 2023
 *      Author: hasan
 */

#ifndef INC_BASICAPP_H_
#define INC_BASICAPP_H_

#include "torkTypes.h"
#include "stm32g0xx_hal.h"
#include "mcp2515_drv.h"
#include "canMsg.h"
#include "canMsgRingBuf.h"

/**
 *
 */
typedef struct
{
	bool is_can_enable;
	MCP_BITTIME_SETUP can_speed;
}Can_Eeprom_Values_t;


/**
 *
 */
typedef struct
{
	bool Is_Route_Enable;

	torkCanMsgRingBuf_t Route_Ring_Buf;
	torkCanMsg Can_Msg_Queue[MAX_BUFFER_DEPTH]; // to store can msg arrays for ring buffers

	_Bool Is_There_Comparison;            // son gönderilen veri ile karsılastırma var mı?
	torkCanMsg Penultimate_Sent_Message;  // ringbuffer'dan poplanan sondan bir önceki mesaj saklanacak
	torkCanMsg Last_Sent_Message;         // ringbuffer'dan poplanan sen son mesaj saklanacak
}Can_Route_Values_t;


void Init_CanA();
void Init_CanB();
void Init_CanC(MCP_BITTIME_SETUP mcp_speed);
void Init_Basic_App();

uint32_t Convert_Can_Length_For_StmLib(uint8_t canLength);
uint8_t  Convert_Can_Length_For_McpLib(uint32_t canLength);

_Bool Compare_Is_Incoming_Message_Different_From_Previous_Message(torkCanMsg newMsg, torkCanMsg oldMsg);
_Bool Compare_Is_Incoming_Message_Different_From_Previous_Two_Message(torkCanMsg newMsg, torkCanMsg oldMsg, torkCanMsg olderMsg);

void Print_Binary(unsigned int num);
void Debug_For_Eeprom_Val();
void ECR_Set_Values();
#endif /* INC_BASICAPP_H_ */
