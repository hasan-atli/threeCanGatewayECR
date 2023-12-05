/*
 * basicApp.c
 *
 *  Created on: Jun 26, 2023
 *      Author: hasan
 */

#include "basicApp.h"
#include "debug.h"
#include "main.h"
#include "mcp2515_hardware.h"
#include "string.h"


/**********************************************************/
// for CanA comm
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_TxHeaderTypeDef txHeader_A;

// for CanB comm
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_TxHeaderTypeDef txHeader_B;

// for CanC comm
CanbusConfig_t cfgCanC;   //spi2
/**********************************************************/

/**********************************************************/


/**********************************************************/

/**
 *
 */
/*****************************************************************************/
// can icin eepromda kayıtlı degerlirin tutuldugu degisken tanımı
Can_Eeprom_Values_t canA_Values = {true, CAN_500KBPS};
Can_Eeprom_Values_t canB_Values = {true, CAN_500KBPS};
Can_Eeprom_Values_t canC_Values = {true, CAN_500KBPS};
/*****************************************************************************/


/**
 * routeOne :  canA ------|------> canB
 * 			   canC ------|

 * routeTwo :  canB -------|------> canA
 * 						   |------> canC
 */
/*****************************************************************************/
Can_Route_Values_t routeOne;
Can_Route_Values_t routeTwo;
/*****************************************************************************/



void Init_CanA()
{
	txHeader_A.Identifier = 0x7FF;
	txHeader_A.IdType = FDCAN_STANDARD_ID;
	txHeader_A.TxFrameType = FDCAN_DATA_FRAME;
	txHeader_A.DataLength = FDCAN_DLC_BYTES_8;
	txHeader_A.ErrorStateIndicator = FDCAN_ESI_PASSIVE;     //...
	txHeader_A.BitRateSwitch = FDCAN_BRS_OFF;               //...
	txHeader_A.FDFormat = FDCAN_CLASSIC_CAN;                //...
	txHeader_A.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     //...
	txHeader_A.MessageMarker = 0;


	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		dbgPrint("ERROR: hfdcan1, HAL_FDCAN_Start\n");
		Error_Handler();
	}

    // Enable interrupt, FIFO0,  FDCAN1, new data
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      dbgPrint("ERROR: hfdcan1, HAL_FDCAN_ActivateNotification\n");
      Error_Handler();
    }

    debugPrint("Init_CanA OK\n");
}



void Init_CanB()
{
	txHeader_B.Identifier = 0x7FF;
	txHeader_B.IdType = FDCAN_STANDARD_ID;
	txHeader_B.TxFrameType = FDCAN_DATA_FRAME;
	txHeader_B.DataLength = FDCAN_DLC_BYTES_8;
	txHeader_B.ErrorStateIndicator = FDCAN_ESI_PASSIVE;     //...
	txHeader_B.BitRateSwitch = FDCAN_BRS_OFF;               //...
	txHeader_B.FDFormat = FDCAN_CLASSIC_CAN;                //...
	txHeader_B.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     //...
	txHeader_B.MessageMarker = 0;

	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		dbgPrint("ERROR: HAL_FDCAN_ConfigGlobalFilter\n");
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		dbgPrint("ERROR: hfdcan2, HAL_FDCAN_Start\n");
		Error_Handler();
	}

	if (HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_GROUP_RX_FIFO1, FDCAN_INTERRUPT_LINE1) != HAL_OK)
	{
		dbgPrint("ERROR: hfdcan2, HAL_FDCAN_ConfigInterruptLines\n");
		Error_Handler();
	}

    // Enable interrupt, FIFO0,  FDCAN1, new data
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
    {
      dbgPrint("ERROR: hfdcan2, HAL_FDCAN_ActivateNotification\n");
      Error_Handler();
    }

    debugPrint("Init_CanB OK\n");

}



void Init_CanC(MCP_BITTIME_SETUP mcp_speed)
{
	/****CanC***/
	// declaring  hardware related pointer func.
	cfgCanC.ChipSelectFp = &ChipSelectFpC;
	cfgCanC.ChipUnSelectFp = &ChipUnSelectFpC;
	cfgCanC.SPIReadFp = &SPIReadFpC;
	cfgCanC.SPIReadWriteFp = &SPIReadWriteFpC;
	cfgCanC.SPIWriteFp = &SPIWriteFpC;

	// init
	ChipUnSelectFpC();

	uint8_t result = begin(&cfgCanC, mcp_speed, MCP_8MHz);
	//can the CAN run
	if (result == CAN_OK) {
		debugPrint("Init_CanC OK\n");
	}
	else
	{
		Error_Handler();
		debugPrint("CAN_C_FAIL\n");
	}
}


void Init_Basic_App()
{
	// for MCP2515
	Init_CanA();
	Init_CanB();
	Init_CanC(canC_Values.can_speed);

	// for RingBuffer
	canMsgRingBufferInit(&routeOne.Route_Ring_Buf, routeOne.Can_Msg_Queue, MAX_BUFFER_DEPTH);
	canMsgRingBufferInit(&routeTwo.Route_Ring_Buf, routeTwo.Can_Msg_Queue, MAX_BUFFER_DEPTH);
}



/**********************************************************/
//  Name        : Compare_Is_Incoming_Message_Different_From_Previous_Message
//  Parameters  :
//  Returns     : true: gelen mesaj son gönderilen mesajdan farklıdır
//				  false: gelen mesaj  gönderilen son mesaj ile aynıdır
//  Function    :
/*--------------------------------------------------------*/
_Bool Compare_Is_Incoming_Message_Different_From_Previous_Message(torkCanMsg newMsg, torkCanMsg oldMsg)
{
	_Bool areIdsDifferent = false;
	_Bool areExtsDifferent = false;
	_Bool areRtrsDifferent = false;
	_Bool areLensDifferent = false;
	_Bool areData1Different = false;
	_Bool areData2Different = false;
	_Bool areData3Different = false;
	_Bool areData4Different = false;


	if((newMsg.Identifier != oldMsg.Identifier))
	{
		areIdsDifferent = true;
	}

	if((newMsg.IdType != oldMsg.IdType))
	{
		areExtsDifferent = true;
	}

	if((newMsg.FrameType != oldMsg.FrameType))
	{
		areRtrsDifferent = true;
	}

	if((newMsg.DataLength != oldMsg.DataLength))
	{
		areLensDifferent = true;
	}


	if(newMsg.Payload[CAN_MSG_DATA_SENDER_ADDR_INDEX] != oldMsg.Payload[CAN_MSG_DATA_SENDER_ADDR_INDEX])
	{
		areData1Different = true;
	}

	if(newMsg.Payload[CAN_MSG_DATA_TIMESTAMP_INDEX] != oldMsg.Payload[CAN_MSG_DATA_TIMESTAMP_INDEX])
	{

		areData2Different = true;
	}

	if(newMsg.Payload[CAN_MSG_DATA_PAYLOAD_0_INDEX] != oldMsg.Payload[CAN_MSG_DATA_PAYLOAD_0_INDEX])
	{
		areData3Different = true;
	}

	if(newMsg.Payload[CAN_MSG_DATA_PAYLOAD_1_INDEX] != oldMsg.Payload[CAN_MSG_DATA_PAYLOAD_1_INDEX])
	{
		areData4Different = true;
	}


	if(areIdsDifferent || areExtsDifferent || areRtrsDifferent || areLensDifferent || areData1Different || areData2Different || areData3Different || areData4Different)
	{
		return true;
	}
	else
	{
		return false;
	}
}



uint32_t Convert_Can_Length_For_StmLib(uint8_t canLength)
{
	switch (canLength)
	{
		case 0:
			return FDCAN_DLC_BYTES_0;

		case 1:
			return FDCAN_DLC_BYTES_1;

		case 2:
			return FDCAN_DLC_BYTES_2;

		case 3:
			return FDCAN_DLC_BYTES_3;

		case 4:
			return FDCAN_DLC_BYTES_4;

		case 5:
			return FDCAN_DLC_BYTES_5;

		case 6:
			return FDCAN_DLC_BYTES_6;

		case 7:
			return FDCAN_DLC_BYTES_7;

		case 8:
			return FDCAN_DLC_BYTES_8;

		default:
			return FDCAN_DLC_BYTES_0;
	}
}


uint8_t  Convert_Can_Length_For_McpLib(uint32_t canLength)
{
	switch (canLength)
	{
	case FDCAN_DLC_BYTES_0:
		return 0;

	case FDCAN_DLC_BYTES_1:
		return 1;

	case FDCAN_DLC_BYTES_2:
		return 2;

	case FDCAN_DLC_BYTES_3:
		return 3;

	case FDCAN_DLC_BYTES_4:
		return 4;

	case FDCAN_DLC_BYTES_5:
		return 5;

	case FDCAN_DLC_BYTES_6:
		return 6;

	case FDCAN_DLC_BYTES_7:
		return 7;

	case FDCAN_DLC_BYTES_8:
		return 8;

	default:
		return 0;
	}
}
