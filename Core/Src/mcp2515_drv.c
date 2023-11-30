/*
 * mco2515_drv.c
 *
 *  Created on: 16 Ağu 2022
 *      Author: BilgeKaan
 */
#include "mcp2515_drv.h"
#include "main.h"
#include "debug.h"

/*********************************************************************************************************
** Function name:           txCtrlReg
** Descriptions:            return tx ctrl reg according to tx buffer index.
**                          According to my tests this is faster and saves memory compared using vector
*********************************************************************************************************/
uint8_t txCtrlReg(uint8_t i)
{
    switch (i) {
        case 0: return MCP_TXB0CTRL;
        case 1: return MCP_TXB1CTRL;
        case 2: return MCP_TXB2CTRL;
    }
    return MCP_TXB2CTRL;
}

/*********************************************************************************************************
** Function name:           statusToBuffer
** Descriptions:            converts CANINTF status to tx buffer index
*********************************************************************************************************/
uint8_t statusToTxBuffer(uint8_t status)
 {
    switch (status) {
        case MCP_TX0IF : return 0;
        case MCP_TX1IF : return 1;
        case MCP_TX2IF : return 2;
    }

    return 0xff;
}

/*********************************************************************************************************
** Function name:           statusToBuffer
** Descriptions:            converts CANINTF status to tx buffer sidh
*********************************************************************************************************/
uint8_t statusToTxSidh(uint8_t status)
 {
    switch (status) {
        case MCP_TX0IF : return MCP_TXB0SIDH;
        case MCP_TX1IF : return MCP_TXB1SIDH;
        case MCP_TX2IF : return MCP_TXB2SIDH;
    }

    return 0;
}

/*********************************************************************************************************
** Function name:           txSidhToTxLoad
** Descriptions:            return tx load command according to tx buffer sidh register
*********************************************************************************************************/
uint8_t txSidhToRTS(uint8_t sidh)
{
    switch (sidh) {
        case MCP_TXB0SIDH: return MCP_RTS_TX0;
        case MCP_TXB1SIDH: return MCP_RTS_TX1;
        case MCP_TXB2SIDH: return MCP_RTS_TX2;
    }
    return 0;
}

/*********************************************************************************************************
** Function name:           txSidhToTxLoad
** Descriptions:            return tx load command according to tx buffer sidh register
*********************************************************************************************************/
uint8_t txSidhToTxLoad(uint8_t sidh)
{
    switch (sidh) {
        case MCP_TXB0SIDH: return MCP_LOAD_TX0;
        case MCP_TXB1SIDH: return MCP_LOAD_TX1;
        case MCP_TXB2SIDH: return MCP_LOAD_TX2;
    }
    return 0;
}

/*********************************************************************************************************
** Function name:           txIfFlag
** Descriptions:            return tx interrupt flag
*********************************************************************************************************/
uint8_t txIfFlag(uint8_t i)
 {
    switch (i)
	{
        case 0: return MCP_TX0IF;
        case 1: return MCP_TX1IF;
        case 2: return MCP_TX2IF;
    }
    return 0;
}

/*********************************************************************************************************
** Function name:           txStatusPendingFlag
** Descriptions:            return buffer tx pending flag on status
*********************************************************************************************************/
uint8_t txStatusPendingFlag(uint8_t i)
 {
    switch (i)
	{
        case 0: return MCP_STAT_TX0_PENDING;
        case 1: return MCP_STAT_TX1_PENDING;
        case 2: return MCP_STAT_TX2_PENDING;
    }
    return 0xff;
}

/*********************************************************************************************************
** Function name:           mcp2515_reset
** Descriptions:            reset the device
*********************************************************************************************************/
void mcp2515_reset(CanbusConfig_t* canCfg)
{
    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(MCP_RESET);
    canCfg->ChipUnSelectFp();

    HW_DELAY(10);
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegister
** Descriptions:            read register
*********************************************************************************************************/
uint8_t mcp2515_readRegister(CanbusConfig_t* canCfg,const uint8_t address)
 {
    uint8_t ret;

    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(MCP_READ);
    canCfg->SPIReadWriteFp(address);
    ret = canCfg->SPIReadFp();
    canCfg->ChipUnSelectFp();

    return ret;
}

/*********************************************************************************************************
** Function name:           mcp2515_readRegisterS
** Descriptions:            read registerS
*********************************************************************************************************/
void mcp2515_readRegisterS(CanbusConfig_t* canCfg,const uint8_t address, uint8_t values[], const uint8_t n)
{
    uint8_t i;

    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(MCP_READ);
    canCfg->SPIReadWriteFp(address);
    // mcp2515 has auto-increment of address-pointer
    for (i = 0; i < n && i < CAN_MSG_MAX_DATA_LENGTH; i++)
	{
        values[i] = canCfg->SPIReadFp();
    }
    canCfg->ChipUnSelectFp();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegister
** Descriptions:            set register
*********************************************************************************************************/
void mcp2515_setRegister(CanbusConfig_t* canCfg,const uint8_t address, const uint8_t value)
{
    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(MCP_WRITE);
    canCfg->SPIReadWriteFp(address);
    canCfg->SPIReadWriteFp(value);
    canCfg->ChipUnSelectFp();
}

/*********************************************************************************************************
** Function name:           mcp2515_setRegisterS
** Descriptions:            set registerS
*********************************************************************************************************/
void mcp2515_setRegisterS(CanbusConfig_t* canCfg,const uint8_t address, const uint8_t values[], const uint8_t n)
{
    uint8_t i;

    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(MCP_WRITE);
    canCfg->SPIReadWriteFp(address);

    for(i = 0; i < n; i++)
	{
        canCfg->SPIReadWriteFp(values[i]);
    }

    canCfg->ChipUnSelectFp();
}

/*********************************************************************************************************
** Function name:           mcp2515_modifyRegister
** Descriptions:            set bit of one register
*********************************************************************************************************/
void mcp2515_modifyRegister(CanbusConfig_t* canCfg,const uint8_t address, const uint8_t mask, const uint8_t data)
{
    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(MCP_BITMOD);
    canCfg->SPIReadWriteFp(address);
    canCfg->SPIReadWriteFp(mask);
    canCfg->SPIReadWriteFp(data);
    canCfg->ChipUnSelectFp();
}

/*********************************************************************************************************
** Function name:           mcp2515_readStatus
** Descriptions:            read mcp2515's Status
*********************************************************************************************************/
uint8_t mcp2515_readStatus(CanbusConfig_t* canCfg)
{
    uint8_t i;

    canCfg->ChipSelectFp();

    canCfg->SPIReadWriteFp(MCP_READ_STATUS);

    i = canCfg->SPIReadFp();

    canCfg->ChipUnSelectFp();

    return i;
}

/*********************************************************************************************************
** Function name:           setSleepWakeup
** Descriptions:            Enable or disable the wake up interrupt (If disabled the MCP2515 will not be woken up by CAN bus activity)
*********************************************************************************************************/
void setSleepWakeup(CanbusConfig_t* canCfg,const uint8_t enable)
{
    mcp2515_modifyRegister(canCfg,MCP_CANINTE, MCP_WAKIF, enable ? MCP_WAKIF : 0);
}

/*********************************************************************************************************
** Function name:           sleep
** Descriptions:            Put mcp2515 in sleep mode to save power
*********************************************************************************************************/
uint8_t sleep(CanbusConfig_t* canCfg)
{
    if (getMode(canCfg) != MODE_SLEEP)
	{
        return mcp2515_setCANCTRL_Mode(canCfg,MODE_SLEEP);
    }
	else
	{
        return CAN_OK;
    }
}

/*********************************************************************************************************
** Function name:           wake
** Descriptions:            wake MCP2515 manually from sleep. It will come back in the mode it was before sleeping.
*********************************************************************************************************/
uint8_t wake(CanbusConfig_t* canCfg)
 {
    uint8_t currMode = getMode(canCfg);

    if (currMode != canCfg->mcpMode)
	{
        return mcp2515_setCANCTRL_Mode(canCfg,canCfg->mcpMode);
    }
	else
	{
        return CAN_OK;
    }
}

/*********************************************************************************************************
** Function name:           setMode
** Descriptions:            Sets control mode
*********************************************************************************************************/
uint8_t setMode(CanbusConfig_t* canCfg,const uint8_t opMode)
 {
    if (opMode != MODE_SLEEP)
	{ // if going to sleep, the value stored in opMode is not changed so that we can return to it later
    	canCfg->mcpMode = opMode;
    }

    return mcp2515_setCANCTRL_Mode(canCfg,opMode);
}

/*********************************************************************************************************
** Function name:           getMode
** Descriptions:            Returns current control mode
*********************************************************************************************************/
uint8_t getMode(CanbusConfig_t* canCfg)
{
    return (mcp2515_readRegister(canCfg,MCP_CANSTAT) & MODE_MASK);
}

/*********************************************************************************************************
** Function name:           mcp2515_setCANCTRL_Mode
** Descriptions:            set control mode
*********************************************************************************************************/
uint8_t mcp2515_setCANCTRL_Mode(CanbusConfig_t* canCfg,const uint8_t newmode)
 {
    // If the chip is asleep and we want to change mode then a manual wake needs to be done
    // This is done by setting the wake up interrupt flag
    // This undocumented trick was found at https://github.com/mkleemann/can/blob/master/can_sleep_mcp2515.c


    if ((getMode(canCfg)) == MODE_SLEEP && (newmode != MODE_SLEEP))
	{
        // Make sure wake interrupt is enabled
        uint8_t wakeIntEnabled = (mcp2515_readRegister(canCfg,MCP_CANINTE) & MCP_WAKIF);

        if (!wakeIntEnabled)
		{
            mcp2515_modifyRegister(canCfg,MCP_CANINTE, MCP_WAKIF, MCP_WAKIF);
        }

        // Set wake flag (this does the actual waking up)
        mcp2515_modifyRegister(canCfg,MCP_CANINTF, MCP_WAKIF, MCP_WAKIF);

        // Wait for the chip to exit SLEEP and enter LISTENONLY mode.

        // If the chip is not connected to a CAN bus (or the bus has no other powered nodes) it will sometimes trigger the wake interrupt as soon
        // as it's put to sleep, but it will stay in SLEEP mode instead of automatically switching to LISTENONLY mode.
        // In this situation the mode needs to be manually set to LISTENONLY.

        if (mcp2515_requestNewMode(canCfg,MODE_LISTENONLY) != MCP2515_OK)
		{
            return MCP2515_FAIL;
        }

        // Turn wake interrupt back off if it was originally off
        if (!wakeIntEnabled)
		{
            mcp2515_modifyRegister(canCfg,MCP_CANINTE, MCP_WAKIF, 0);
        }
    }

    // Clear wake flag
    mcp2515_modifyRegister(canCfg,MCP_CANINTF, MCP_WAKIF, 0);

    return mcp2515_requestNewMode(canCfg,newmode);
}

/*********************************************************************************************************
** Function name:           mcp2515_requestNewMode
** Descriptions:            Set control mode
*********************************************************************************************************/
uint8_t mcp2515_requestNewMode(CanbusConfig_t* canCfg,const uint8_t newmode)
{
    unsigned long startTime = GET_TICK_MS();

    // Spam new mode request and wait for the operation  to complete
    while (1)
	{
        // Request new mode
        // This is inside the loop as sometimes requesting the new mode once doesn't work (usually when attempting to sleep)
        mcp2515_modifyRegister(canCfg,MCP_CANCTRL, MODE_MASK, newmode);

        uint8_t statReg = mcp2515_readRegister(canCfg,MCP_CANSTAT);

        if ((statReg & MODE_MASK) == newmode)
		{ // We're now in the new mode
            return MCP2515_OK;
        }
		else if ((GET_TICK_MS() - startTime) > 200)
		{ // Wait no more than 200ms for the operation to complete
            return MCP2515_FAIL;
        }
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_configRate
** Descriptions:            set baudrate
*********************************************************************************************************/
uint8_t mcp2515_configRate(CanbusConfig_t* canCfg,const uint8_t canSpeed, const uint8_t clock)
 {
    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (clock) {
        case (MCP_16MHz) :
            switch (canSpeed) {
                case (CAN_5KBPS):
                    cfg1 = MCP_16MHz_5kBPS_CFG1;
                    cfg2 = MCP_16MHz_5kBPS_CFG2;
                    cfg3 = MCP_16MHz_5kBPS_CFG3;
                    break;

                case (CAN_10KBPS):
                    cfg1 = MCP_16MHz_10kBPS_CFG1;
                    cfg2 = MCP_16MHz_10kBPS_CFG2;
                    cfg3 = MCP_16MHz_10kBPS_CFG3;
                    break;

                case (CAN_20KBPS):
                    cfg1 = MCP_16MHz_20kBPS_CFG1;
                    cfg2 = MCP_16MHz_20kBPS_CFG2;
                    cfg3 = MCP_16MHz_20kBPS_CFG3;
                    break;

                case (CAN_25KBPS):
                    cfg1 = MCP_16MHz_25kBPS_CFG1;
                    cfg2 = MCP_16MHz_25kBPS_CFG2;
                    cfg3 = MCP_16MHz_25kBPS_CFG3;
                    break;

                case (CAN_31K25BPS):
                    cfg1 = MCP_16MHz_31k25BPS_CFG1;
                    cfg2 = MCP_16MHz_31k25BPS_CFG2;
                    cfg3 = MCP_16MHz_31k25BPS_CFG3;
                    break;

                case (CAN_33KBPS):
                    cfg1 = MCP_16MHz_33kBPS_CFG1;
                    cfg2 = MCP_16MHz_33kBPS_CFG2;
                    cfg3 = MCP_16MHz_33kBPS_CFG3;
                    break;

                case (CAN_40KBPS):
                    cfg1 = MCP_16MHz_40kBPS_CFG1;
                    cfg2 = MCP_16MHz_40kBPS_CFG2;
                    cfg3 = MCP_16MHz_40kBPS_CFG3;
                    break;

                case (CAN_50KBPS):
                    cfg1 = MCP_16MHz_50kBPS_CFG1;
                    cfg2 = MCP_16MHz_50kBPS_CFG2;
                    cfg3 = MCP_16MHz_50kBPS_CFG3;
                    break;

                case (CAN_80KBPS):
                    cfg1 = MCP_16MHz_80kBPS_CFG1;
                    cfg2 = MCP_16MHz_80kBPS_CFG2;
                    cfg3 = MCP_16MHz_80kBPS_CFG3;
                    break;

                case (CAN_83K3BPS):
                    cfg1 = MCP_16MHz_83k3BPS_CFG1;
                    cfg2 = MCP_16MHz_83k3BPS_CFG2;
                    cfg3 = MCP_16MHz_83k3BPS_CFG3;
                    break;

                case (CAN_95KBPS):
                    cfg1 = MCP_16MHz_95kBPS_CFG1;
                    cfg2 = MCP_16MHz_95kBPS_CFG2;
                    cfg3 = MCP_16MHz_95kBPS_CFG3;
                    break;

                case (CAN_100KBPS):
                    cfg1 = MCP_16MHz_100kBPS_CFG1;
                    cfg2 = MCP_16MHz_100kBPS_CFG2;
                    cfg3 = MCP_16MHz_100kBPS_CFG3;
                    break;

                case (CAN_125KBPS):
                    cfg1 = MCP_16MHz_125kBPS_CFG1;
                    cfg2 = MCP_16MHz_125kBPS_CFG2;
                    cfg3 = MCP_16MHz_125kBPS_CFG3;
                    break;

                case (CAN_200KBPS):
                    cfg1 = MCP_16MHz_200kBPS_CFG1;
                    cfg2 = MCP_16MHz_200kBPS_CFG2;
                    cfg3 = MCP_16MHz_200kBPS_CFG3;
                    break;

                case (CAN_250KBPS):
                    cfg1 = MCP_16MHz_250kBPS_CFG1;
                    cfg2 = MCP_16MHz_250kBPS_CFG2;
                    cfg3 = MCP_16MHz_250kBPS_CFG3;
                    break;

                case (CAN_500KBPS):
                    cfg1 = MCP_16MHz_500kBPS_CFG1;
                    cfg2 = MCP_16MHz_500kBPS_CFG2;
                    cfg3 = MCP_16MHz_500kBPS_CFG3;
                    break;

                case (CAN_666KBPS):
                    cfg1 = MCP_16MHz_666kBPS_CFG1;
                    cfg2 = MCP_16MHz_666kBPS_CFG2;
                    cfg3 = MCP_16MHz_666kBPS_CFG3;
                    break;

                case (CAN_800KBPS) :
                    cfg1 = MCP_16MHz_800kBPS_CFG1;
                    cfg2 = MCP_16MHz_800kBPS_CFG2;
                    cfg3 = MCP_16MHz_800kBPS_CFG3;
                    break;

                case (CAN_1000KBPS):
                    cfg1 = MCP_16MHz_1000kBPS_CFG1;
                    cfg2 = MCP_16MHz_1000kBPS_CFG2;
                    cfg3 = MCP_16MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;

        case (MCP_12MHz) :
            switch (canSpeed) {
                case (CAN_20KBPS) :
                    cfg1 = MCP_12MHz_20kBPS_CFG1;
                    cfg2 = MCP_12MHz_20kBPS_CFG2;
                    cfg3 = MCP_12MHz_20kBPS_CFG3;
                    break;

                case (CAN_25KBPS) :
                    cfg1 = MCP_12MHz_25kBPS_CFG1;
                    cfg2 = MCP_12MHz_25kBPS_CFG2;
                    cfg3 = MCP_12MHz_25kBPS_CFG3;
                    break;

                case (CAN_31K25BPS) :
                    cfg1 = MCP_12MHz_31k25BPS_CFG1;
                    cfg2 = MCP_12MHz_31k25BPS_CFG2;
                    cfg3 = MCP_12MHz_31k25BPS_CFG3;
                    break;

                case (CAN_33KBPS) :
                    cfg1 = MCP_12MHz_33kBPS_CFG1;
                    cfg2 = MCP_12MHz_33kBPS_CFG2;
                    cfg3 = MCP_12MHz_33kBPS_CFG3;
                    break;

                case (CAN_40KBPS) :
                    cfg1 = MCP_12MHz_40kBPS_CFG1;
                    cfg2 = MCP_12MHz_40kBPS_CFG2;
                    cfg3 = MCP_12MHz_40kBPS_CFG3;
                    break;

                case (CAN_50KBPS) :
                    cfg1 = MCP_12MHz_50kBPS_CFG1;
                    cfg2 = MCP_12MHz_50kBPS_CFG2;
                    cfg3 = MCP_12MHz_50kBPS_CFG3;
                    break;

                case (CAN_80KBPS) :
                    cfg1 = MCP_12MHz_80kBPS_CFG1;
                    cfg2 = MCP_12MHz_80kBPS_CFG2;
                    cfg3 = MCP_12MHz_80kBPS_CFG3;
                    break;

                case (CAN_83K3BPS) :
                    cfg1 = MCP_12MHz_83k3BPS_CFG1;
                    cfg2 = MCP_12MHz_83k3BPS_CFG2;
                    cfg3 = MCP_12MHz_83k3BPS_CFG3;
                    break;

                case (CAN_95KBPS) :
                    cfg1 = MCP_12MHz_95kBPS_CFG1;
                    cfg2 = MCP_12MHz_95kBPS_CFG2;
                    cfg3 = MCP_12MHz_95kBPS_CFG3;
                    break;

                case (CAN_100KBPS) :
                    cfg1 = MCP_12MHz_100kBPS_CFG1;
                    cfg2 = MCP_12MHz_100kBPS_CFG2;
                    cfg3 = MCP_12MHz_100kBPS_CFG3;
                    break;

                case (CAN_125KBPS) :
                    cfg1 = MCP_12MHz_125kBPS_CFG1;
                    cfg2 = MCP_12MHz_125kBPS_CFG2;
                    cfg3 = MCP_12MHz_125kBPS_CFG3;
                    break;

                case (CAN_200KBPS) :
                    cfg1 = MCP_12MHz_200kBPS_CFG1;
                    cfg2 = MCP_12MHz_200kBPS_CFG2;
                    cfg3 = MCP_12MHz_200kBPS_CFG3;
                    break;

                case (CAN_250KBPS) :
                    cfg1 = MCP_12MHz_250kBPS_CFG1;
                    cfg2 = MCP_12MHz_250kBPS_CFG2;
                    cfg3 = MCP_12MHz_250kBPS_CFG3;
                    break;

                case (CAN_500KBPS) :
                    cfg1 = MCP_12MHz_500kBPS_CFG1;
                    cfg2 = MCP_12MHz_500kBPS_CFG2;
                    cfg3 = MCP_12MHz_500kBPS_CFG3;
                    break;

                case (CAN_666KBPS) :
                    cfg1 = MCP_12MHz_666kBPS_CFG1;
                    cfg2 = MCP_12MHz_666kBPS_CFG2;
                    cfg3 = MCP_12MHz_666kBPS_CFG3;
                    break;

                case (CAN_1000KBPS) :
                    cfg1 = MCP_12MHz_1000kBPS_CFG1;
                    cfg2 = MCP_12MHz_1000kBPS_CFG2;
                    cfg3 = MCP_12MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;
        case (MCP_8MHz) :
            switch (canSpeed) {
                case (CAN_5KBPS) :
                    cfg1 = MCP_8MHz_5kBPS_CFG1;
                    cfg2 = MCP_8MHz_5kBPS_CFG2;
                    cfg3 = MCP_8MHz_5kBPS_CFG3;
                    break;

                case (CAN_10KBPS) :
                    cfg1 = MCP_8MHz_10kBPS_CFG1;
                    cfg2 = MCP_8MHz_10kBPS_CFG2;
                    cfg3 = MCP_8MHz_10kBPS_CFG3;
                    break;

                case (CAN_20KBPS) :
                    cfg1 = MCP_8MHz_20kBPS_CFG1;
                    cfg2 = MCP_8MHz_20kBPS_CFG2;
                    cfg3 = MCP_8MHz_20kBPS_CFG3;
                    break;

                case (CAN_31K25BPS) :
                    cfg1 = MCP_8MHz_31k25BPS_CFG1;
                    cfg2 = MCP_8MHz_31k25BPS_CFG2;
                    cfg3 = MCP_8MHz_31k25BPS_CFG3;
                    break;

                case (CAN_40KBPS) :
                    cfg1 = MCP_8MHz_40kBPS_CFG1;
                    cfg2 = MCP_8MHz_40kBPS_CFG2;
                    cfg3 = MCP_8MHz_40kBPS_CFG3;
                    break;

                case (CAN_50KBPS) :
                    cfg1 = MCP_8MHz_50kBPS_CFG1;
                    cfg2 = MCP_8MHz_50kBPS_CFG2;
                    cfg3 = MCP_8MHz_50kBPS_CFG3;
                    break;

                case (CAN_80KBPS) :
                    cfg1 = MCP_8MHz_80kBPS_CFG1;
                    cfg2 = MCP_8MHz_80kBPS_CFG2;
                    cfg3 = MCP_8MHz_80kBPS_CFG3;
                    break;

                case (CAN_100KBPS) :
                    cfg1 = MCP_8MHz_100kBPS_CFG1;
                    cfg2 = MCP_8MHz_100kBPS_CFG2;
                    cfg3 = MCP_8MHz_100kBPS_CFG3;
                    break;

                case (CAN_125KBPS) :
                    cfg1 = MCP_8MHz_125kBPS_CFG1;
                    cfg2 = MCP_8MHz_125kBPS_CFG2;
                    cfg3 = MCP_8MHz_125kBPS_CFG3;
                    break;

                case (CAN_200KBPS) :
                    cfg1 = MCP_8MHz_200kBPS_CFG1;
                    cfg2 = MCP_8MHz_200kBPS_CFG2;
                    cfg3 = MCP_8MHz_200kBPS_CFG3;
                    break;

                case (CAN_250KBPS) :
                    cfg1 = MCP_8MHz_250kBPS_CFG1;
                    cfg2 = MCP_8MHz_250kBPS_CFG2;
                    cfg3 = MCP_8MHz_250kBPS_CFG3;
                    break;

                case (CAN_500KBPS) :
                    cfg1 = MCP_8MHz_500kBPS_CFG1;
                    cfg2 = MCP_8MHz_500kBPS_CFG2;
                    cfg3 = MCP_8MHz_500kBPS_CFG3;
                    break;

                case (CAN_800KBPS) :
                    cfg1 = MCP_8MHz_800kBPS_CFG1;
                    cfg2 = MCP_8MHz_800kBPS_CFG2;
                    cfg3 = MCP_8MHz_800kBPS_CFG3;
                    break;

                case (CAN_1000KBPS) :
                    cfg1 = MCP_8MHz_1000kBPS_CFG1;
                    cfg2 = MCP_8MHz_1000kBPS_CFG2;
                    cfg3 = MCP_8MHz_1000kBPS_CFG3;
                    break;

                default:
                    set = 0;
                    break;
            }
            break;

        default:
            set = 0;
            break;
    }

    if (set)
	{
        mcp2515_setRegister(canCfg,MCP_CNF1, cfg1);
        mcp2515_setRegister(canCfg,MCP_CNF2, cfg2);
        mcp2515_setRegister(canCfg,MCP_CNF3, cfg3);

        return MCP2515_OK;
    }
	else
	{
        return MCP2515_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_initCANBuffers
** Descriptions:            init canbuffers
*********************************************************************************************************/
void mcp2515_initCANBuffers(CanbusConfig_t* canCfg)
{
    uint8_t i, a1, a2, a3;

    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++)
		{                       // in-buffer loop
        mcp2515_setRegister(canCfg,a1,0);
        mcp2515_setRegister(canCfg,a2,0);
        mcp2515_setRegister(canCfg,a3,0);
        a1++;
        a2++;
        a3++;
    }
    mcp2515_setRegister(canCfg,MCP_RXB0CTRL,0);
    mcp2515_setRegister(canCfg,MCP_RXB1CTRL,0);
}

/*********************************************************************************************************
** Function name:           mcp2515_init
** Descriptions:            init the device
*********************************************************************************************************/
uint8_t mcp2515_init(CanbusConfig_t* canCfg,const uint8_t canSpeed, const uint8_t clock)
 {
    uint8_t res;

    mcp2515_reset(canCfg);
    res = mcp2515_setCANCTRL_Mode(canCfg,MODE_CONFIG);

    if (res > 0)
	{
        #if DEBUG_EN
        debugPrint("Enter setting mode fail\n");
        #else
        HW_DELAY(10);
        #endif
        return res;
    }
    #if DEBUG_EN
    debugPrint("Enter setting mode success\n ");
    #else
    HW_DELAY(10);
    #endif

    // set boadrate
    if (mcp2515_configRate(canCfg,canSpeed, clock)) {
        #if DEBUG_EN
    	debugPrint("set rate fall!!\n");
        #else
        HW_DELAY(10);
        #endif
        return res;
    }
    #if DEBUG_EN
    debugPrint("set rate success!!\n");
    #else
    HW_DELAY(10);
    #endif

    if (res == MCP2515_OK) {

        // init canbuffers
        mcp2515_initCANBuffers(canCfg);

        // interrupt mode
        mcp2515_setRegister(canCfg,MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);

        #if (DEBUG_RXANY==1)
        // enable both receive-buffers to receive any message and enable rollover
        mcp2515_modifyRegister(canCfg,MCP_RXB0CTRL,MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
        mcp2515_modifyRegister(canCfg,MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_ANY);
        #else


        mcp2515_modifyRegister(canCfg,MCP_RXB0CTRL,MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
        mcp2515_modifyRegister(canCfg,MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_STDEXT);
        #endif
        // enter normal mode
        res = setMode(canCfg,MODE_NORMAL);
        if (res)
		{
            #if DEBUG_EN
            debugPrint("Enter Normal Mode Fail!!\n");
            #else
            HW_DELAY(10);
            #endif
            return res;
        }

        #if DEBUG_EN
        debugPrint("Enter Normal Mode Success!!\n");
        #else
        HW_DELAY(10);
        #endif
    }
    return res;
}

/*********************************************************************************************************
** Function name:           mcp2515_id_to_buf
** Descriptions:            configure tbufdata[4] from id and ext
*********************************************************************************************************/
void mcp2515_id_to_buf(const uint8_t ext, const unsigned long id, uint8_t* tbufdata)
 {
    uint16_t canid;

    canid = (uint16_t)(id & 0x0FFFF);

    if (ext == 1)
	{
        tbufdata[MCP_EID0] = (uint8_t)(canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t)(canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t)(canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t)((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t)(canid >> 5);
    }
	else
	{
        tbufdata[MCP_SIDH] = (uint8_t)(canid >> 3);
        tbufdata[MCP_SIDL] = (uint8_t)((canid & 0x07) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_id
** Descriptions:            write can id
*********************************************************************************************************/
void mcp2515_write_id(CanbusConfig_t* canCfg,const uint8_t mcp_addr, const uint8_t ext, const unsigned long id)
{
    uint8_t tbufdata[4];

    mcp2515_id_to_buf(ext,id,tbufdata);
    mcp2515_setRegisterS(canCfg,mcp_addr,tbufdata,4);
}

/*********************************************************************************************************
** Function name:           mcp2515_read_id
** Descriptions:            read can id
*********************************************************************************************************/
void mcp2515_read_id(CanbusConfig_t* canCfg,const uint8_t mcp_addr, uint8_t* ext, unsigned long* id)
 {
    uint8_t tbufdata[4];

    *ext    = 0;
    *id     = 0;

    mcp2515_readRegisterS(canCfg,mcp_addr,tbufdata,4);

    *id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);

    if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M)
	{
        // extended id
        *id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id << 8) + tbufdata[MCP_EID8];
        *id = (*id << 8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           mcp2515_write_canMsg
** Descriptions:            write msg
**                          Note! There is no check for right address!
*********************************************************************************************************/
void mcp2515_write_canMsg(CanbusConfig_t* canCfg,const uint8_t buffer_sidh_addr, unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len,volatile const uint8_t* buf)
{
    uint8_t load_addr = txSidhToTxLoad(buffer_sidh_addr);

    uint8_t tbufdata[4];
    uint8_t dlc = len | (rtrBit ? MCP_RTR_MASK : 0) ;
    uint8_t i;

    ///////////////////////////////////////
    if(rtrBit == 1){	//RTR LEN UPDATE
    len = 0;
    }
    ///////////////////////////////////////

    mcp2515_id_to_buf(ext, id, tbufdata);

    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(load_addr);

    for (i = 0; i < 4; i++)
	{
         canCfg->SPIWriteFp(tbufdata[i]);
    }

	canCfg->SPIWriteFp(dlc);

    for (i = 0; i < len && i < CAN_MSG_MAX_DATA_LENGTH; i++)
	{
        canCfg->SPIWriteFp(buf[i]);
    }

    canCfg->ChipUnSelectFp();

    mcp2515_start_transmit(canCfg,buffer_sidh_addr);
}

/*********************************************************************************************************
** Function name:           mcp2515_read_canMsg
** Descriptions:            read message
*********************************************************************************************************/
void mcp2515_read_canMsg(CanbusConfig_t* canCfg,const uint8_t buffer_load_addr, volatile unsigned long* id, volatile uint8_t* ext,volatile uint8_t* rtrBit, volatile uint8_t* len, volatile uint8_t* buf)
 {      /* read can msg                 */
    uint8_t tbufdata[4];
    uint8_t i;

    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(buffer_load_addr);
    // mcp2515 has auto-increment of address-pointer
    for (i = 0; i < 4; i++)
	{
        tbufdata[i] = canCfg->SPIReadFp();
    }


    *id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);
    *ext = 0;
    if ((tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M)
	{
        /* extended id                  */
        *id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
        *id = (*id << 8) + tbufdata[MCP_EID8];
        *id = (*id << 8) + tbufdata[MCP_EID0];
        *ext = 1;
    }

    uint8_t pMsgSize = canCfg->SPIReadFp();

    *len = pMsgSize & MCP_DLC_MASK;

    *rtrBit = (pMsgSize & MCP_RTR_MASK) ? 1 : 0;

    for (i = 0; i < *len && i < CAN_MSG_MAX_DATA_LENGTH; i++)
	{
        buf[i] = canCfg->SPIReadFp();
    }

    canCfg->ChipUnSelectFp();
}

/*********************************************************************************************************
** Function name:           mcp2515_start_transmit
** Descriptions:            Start message transmit on mcp2515
*********************************************************************************************************/
void mcp2515_start_transmit(CanbusConfig_t* canCfg,const uint8_t mcp_addr)
 {            // start transmit
    canCfg->ChipSelectFp();
    canCfg->SPIReadWriteFp(txSidhToRTS(mcp_addr));
    canCfg->ChipUnSelectFp();
}

/*********************************************************************************************************
** Function name:           mcp2515_isTXBufFree
** Descriptions:            Test is tx buffer free for transmitting
*********************************************************************************************************/
uint8_t mcp2515_isTXBufFree(CanbusConfig_t* canCfg,uint8_t* txbuf_n, uint8_t iBuf)
 {         /* get Next free txbuf          */
    *txbuf_n = 0x00;

    if (iBuf >= MCP_N_TXBUFFERS ||(mcp2515_readStatus(canCfg) & txStatusPendingFlag(iBuf)) != 0)
	{
        return MCP_ALLTXBUSY;
    }

    *txbuf_n = txCtrlReg(iBuf) + 1;
	/* return SIDH-address of Buffer */
    mcp2515_modifyRegister(canCfg,MCP_CANINTF, txIfFlag(iBuf), 0);

    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           mcp2515_getNextFreeTXBuf
** Descriptions:            finds next free tx buffer for sending. Return MCP_ALLTXBUSY, if there is none.
*********************************************************************************************************/
uint8_t mcp2515_getNextFreeTXBuf(CanbusConfig_t* canCfg,uint8_t* txbuf_n)
{               // get Next free txbuf
    uint8_t status = mcp2515_readStatus(canCfg) & MCP_STAT_TX_PENDING_MASK;
    uint8_t i;

    *txbuf_n = 0x00;

    if (status == MCP_STAT_TX_PENDING_MASK)
	{
        return MCP_ALLTXBUSY;    // All buffers are pending
    }

    // check all 3 TX-Buffers except reserved
    for (i = 0; i < MCP_N_TXBUFFERS - canCfg->nReservedTx; i++)
	{
        if ((status & txStatusPendingFlag(i)) == 0)
		{
            *txbuf_n = txCtrlReg(i) + 1;                                   // return SIDH-address of Buffer
            mcp2515_modifyRegister(canCfg,MCP_CANINTF, txIfFlag(i), 0);
            return MCP2515_OK;                                                 // ! function exit
        }
    }
    return MCP_ALLTXBUSY;
}
/*********************************************************************************************************
** Function name:           begin
** Descriptions:            init can and set speed
*********************************************************************************************************/
uint8_t begin(CanbusConfig_t* canCfg,uint32_t speedset, const uint8_t clockset)
{

    uint8_t res = mcp2515_init(canCfg,(uint8_t)speedset, clockset);

    return ((res == MCP2515_OK) ? CAN_OK : CAN_FAILINIT);
}

/*********************************************************************************************************
** Function name:           enableTxInterrupt
** Descriptions:            enable interrupt for all tx buffers
*********************************************************************************************************/
void enableTxInterrupt(CanbusConfig_t* canCfg,int8_t enable)
 {
    uint8_t interruptStatus = mcp2515_readRegister(canCfg,MCP_CANINTE);

    if (enable)
	{
        interruptStatus |= MCP_TX_INT;
    }
	else
	{
        interruptStatus &= ~MCP_TX_INT;
    }

    mcp2515_setRegister(canCfg,MCP_CANINTE, interruptStatus);
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            init canid Masks
*********************************************************************************************************/
uint8_t init_Mask(CanbusConfig_t* canCfg,uint8_t num, uint8_t ext, unsigned long ulData)
{
    uint8_t res = MCP2515_OK;
    #if DEBUG_EN
    //DBG_PRINT("Begin to set Mask!!");
    #else
    HW_DELAY(10);
    #endif
    res = mcp2515_setCANCTRL_Mode(canCfg,MODE_CONFIG);

    if (res > 0)
	{
        #if DEBUG_EN
        //DBG_PRINT("Enter setting mode fall");
        #else
        HW_DELAY(10);
        #endif
        return res;
    }

    if (num == 0)
	{
        mcp2515_write_id(canCfg,MCP_RXM0SIDH, ext, ulData);

    }
	else if (num == 1)
	{
        mcp2515_write_id(canCfg,MCP_RXM1SIDH, ext, ulData);
    }
	else
	{
        res =  MCP2515_FAIL;
    }

    res = mcp2515_setCANCTRL_Mode(canCfg,canCfg->mcpMode);

    if (res > 0)
	{
        #if DEBUG_EN
        DBG_PRINT("Enter normal mode fall");
        #else
        HW_DELAY(10);
        #endif
        return res;
    }
    #if DEBUG_EN
    DBG_PRINT("set Mask success!!");
    #else
    HW_DELAY(10);
    #endif
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            init canid filters
*********************************************************************************************************/
uint8_t init_Filt(CanbusConfig_t* canCfg,uint8_t num, uint8_t ext, unsigned long ulData)
{
    uint8_t res = MCP2515_OK;
    #if DEBUG_EN
    DBG_PRINT("Begin to set Filter!!");
    #else
    HW_DELAY(10);
    #endif
    res = mcp2515_setCANCTRL_Mode(canCfg,MODE_CONFIG);

    if (res > 0)
	{
        #if DEBUG_EN
        DBG_PRINT("Enter setting mode fail");
        #else
        	HW_DELAY(10);
        #endif
        return res;
    }

    switch (num)
	{
        case 0:
            mcp2515_write_id(canCfg,MCP_RXF0SIDH, ext, ulData);
            break;

        case 1:
            mcp2515_write_id(canCfg,MCP_RXF1SIDH, ext, ulData);
            break;

        case 2:
            mcp2515_write_id(canCfg,MCP_RXF2SIDH, ext, ulData);
            break;

        case 3:
            mcp2515_write_id(canCfg,MCP_RXF3SIDH, ext, ulData);
            break;

        case 4:
            mcp2515_write_id(canCfg,MCP_RXF4SIDH, ext, ulData);
            break;

        case 5:
            mcp2515_write_id(canCfg,MCP_RXF5SIDH, ext, ulData);
            break;

        default:
            res = MCP2515_FAIL;
    }

    res = mcp2515_setCANCTRL_Mode(canCfg,canCfg->mcpMode);

    if (res > 0)
	{
        #if DEBUG_EN
        DBG_PRINT("Enter normal mode fall\r\nSet filter fail!!");
        #else
        HW_DELAY(10);
        #endif
        return res;
    }
    #if DEBUG_EN
    DBG_PRINT("set Filter success!!");
    #else
    HW_DELAY(10);
    #endif

    return res;
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message by using buffer read as free from CANINTF status
**                          Status has to be read with readRxTxStatus and filtered with checkClearTxStatus
*********************************************************************************************************/
uint8_t sendMsgBuffer(CanbusConfig_t* canCfg,unsigned long id, uint8_t ext,uint8_t rtr, uint8_t len,  uint8_t* buf)
{
	return sendMsg(canCfg,id,ext,rtr,len,buf,true);
}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            Send message by using buffer read as free from CANINTF status
**                          Status has to be read with readRxTxStatus and filtered with checkClearTxStatus
*********************************************************************************************************/
uint8_t sendMsgBuf(CanbusConfig_t* canCfg,uint8_t status, unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, volatile const uint8_t* buf)
{
    uint8_t txbuf_n = statusToTxSidh(status);

    if (txbuf_n == 0)
	{
        return CAN_FAILTX;    // Invalid status
    }

    mcp2515_modifyRegister(canCfg,MCP_CANINTF, status, 0);  // Clear interrupt flag
    mcp2515_write_canMsg(canCfg,txbuf_n,id,ext,rtrBit,len,buf);

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           trySendMsgBuf
** Descriptions:            Try to send message. There is no HW_DELAYs for waiting free buffer.
*********************************************************************************************************/
uint8_t trySendMsgBuf(CanbusConfig_t* canCfg,unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t* buf, uint8_t iTxBuf)
{
    uint8_t txbuf_n;

    if (iTxBuf < MCP_N_TXBUFFERS)
		{ // Use specified buffer
        if (mcp2515_isTXBufFree(canCfg,&txbuf_n, iTxBuf) != MCP2515_OK)
		{
            return CAN_FAILTX;
        }
    }
	else
	{
        if (mcp2515_getNextFreeTXBuf(canCfg,&txbuf_n) != MCP2515_OK)
		{
            return CAN_FAILTX;
        }
    }

    mcp2515_write_canMsg(canCfg,txbuf_n, id, ext, rtrBit, len, buf);

    return CAN_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
*********************************************************************************************************/
uint8_t sendMsg(CanbusConfig_t* canCfg,unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t* buf, int8_t wait_sent)
{
    uint8_t res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    canCfg->can_id  = id;
    canCfg->ext_flg = ext;
    canCfg->rtr     = rtrBit;

    do
    {
        if (uiTimeOut > 0)
		{
            HW_DELAYMICROSECS(10);//
        }
        res = mcp2515_getNextFreeTXBuf(canCfg,&txbuf_n);                       // info = addr.
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if (uiTimeOut == TIMEOUTVALUE)
    {
        return CAN_GETTXBFTIMEOUT;                                      // get tx buff time out
    }
    mcp2515_write_canMsg(canCfg,txbuf_n, id, ext, rtrBit, len, buf);

    if (wait_sent)
    {
        uiTimeOut = 0;
        do
        {
            if (uiTimeOut > 0)
            {
            	HW_DELAYMICROSECS(10);
            }
            uiTimeOut++;
            res1 = mcp2515_readRegister(canCfg,txbuf_n - 1);  // read send buff ctrl reg
            res1 = res1 & 0x08;
        } while (res1 && (uiTimeOut < TIMEOUTVALUE));

        if (uiTimeOut == TIMEOUTVALUE)
        {                                     // send msg timeout
            return CAN_SENDMSGTIMEOUT;
        }
    }

    return CAN_OK;

}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            send buf
*********************************************************************************************************/
uint8_t sendMsgBufWait(CanbusConfig_t* canCfg,unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t* buf, int8_t wait_sent)
{
    return sendMsg(canCfg,id, ext, rtrBit, len, buf, wait_sent);
}

/*********************************************************************************************************
** Function name:           readMsgBufID
** Descriptions:            Read message buf and can bus source ID according to status.
**                          Status has to be read with readRxTxStatus.
*********************************************************************************************************/
uint8_t readMsgBufID(CanbusConfig_t* canCfg,uint8_t status, volatile unsigned long* id, volatile uint8_t* ext, volatile uint8_t* rtrBit,volatile uint8_t* len, volatile uint8_t* buf)
{
    uint8_t rc = CAN_NOMSG;

    if (status & MCP_RX0IF)
    {                                        // Msg in Buffer 0
        mcp2515_read_canMsg(canCfg,MCP_READ_RX0, id, ext, rtrBit, len, buf);
        rc = CAN_OK;
    }
    else if (status & MCP_RX1IF)
    {                                 // Msg in Buffer 1
        mcp2515_read_canMsg(canCfg,MCP_READ_RX1, id, ext, rtrBit, len, buf);
        rc = CAN_OK;
    }

    if (rc == CAN_OK)
    {
    	canCfg->rtr = *rtrBit;
        // dta_len=*len; // not used on any interface function
    	canCfg->ext_flg = *ext;
    	canCfg->can_id = *id;
    } else
    {
        *len = 0;
    }

    return rc;
}


/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            wrapper readMsgBufID func.
**                          Read message buf
*********************************************************************************************************/
uint8_t readMsgBuf(CanbusConfig_t* canCfg, uint8_t *len, uint8_t *buf)
{
	uint8_t status = readRxTxStatus(canCfg);

	return readMsgBufID(canCfg, status, &canCfg->can_id, &canCfg->ext_flg, &canCfg->rtr, len, buf);

}



/*********************************************************************************************************
** Function name:           readRxTxStatus
** Descriptions:            Read RX and TX interrupt bits. Function uses status reading, but translates.
**                          result to MCP_CANINTF. With this you can check status e.g. on interrupt sr
**                          with one single call to save SPI calls. Then use checkClearRxStatus and
**                          checkClearTxStatus for testing.
*********************************************************************************************************/
uint8_t readRxTxStatus(CanbusConfig_t* canCfg) {
    uint8_t ret = (mcp2515_readStatus(canCfg) & (MCP_STAT_TXIF_MASK | MCP_STAT_RXIF_MASK));
    ret = (ret & MCP_STAT_TX0IF ? MCP_TX0IF : 0) |
          (ret & MCP_STAT_TX1IF ? MCP_TX1IF : 0) |
          (ret & MCP_STAT_TX2IF ? MCP_TX2IF : 0) |
          (ret & MCP_STAT_RXIF_MASK); // Rx bits happend to be same on status and MCP_CANINTF
    return ret;
}

/*********************************************************************************************************
** Function name:           checkClearRxStatus
** Descriptions:            Return first found rx CANINTF status and clears it from parameter.
**                          Note that this does not affect to chip CANINTF at all. You can use this
**                          with one single readRxTxStatus call.
*********************************************************************************************************/
uint8_t checkClearRxStatus(uint8_t* status) {
    uint8_t ret;

    ret = *status & MCP_RX0IF; *status &= ~MCP_RX0IF;

    if (ret == 0) {
        ret = *status & MCP_RX1IF;
        *status &= ~MCP_RX1IF;
    }

    return ret;
}

/*********************************************************************************************************
** Function name:           checkClearTxStatus
** Descriptions:            Return specified buffer of first found tx CANINTF status and clears it from parameter.
**                          Note that this does not affect to chip CANINTF at all. You can use this
**                          with one single readRxTxStatus call.
*********************************************************************************************************/
uint8_t checkClearTxStatus(CanbusConfig_t* canCfg,uint8_t* status, uint8_t iTxBuf)
{
    uint8_t ret;

    if (iTxBuf < MCP_N_TXBUFFERS)
    { // Clear specific buffer flag,
        ret = *status & txIfFlag(iTxBuf); *status &= ~txIfFlag(iTxBuf);
    }
    else
    {
        ret = 0;

        for (uint8_t i = 0; i < MCP_N_TXBUFFERS - canCfg->nReservedTx; i++)
        {
            ret = *status & txIfFlag(i);

            if (ret != 0)
            {
                *status &= ~txIfFlag(i);
                return ret;
            }
        }
    }

    return ret;
}

/*********************************************************************************************************
** Function name:           clearBufferTransmitIfFlags
** Descriptions:            Clear transmit interrupt flags for specific buffer or for all unreserved buffers.
**                          If interrupt will be used, it is important to clear all flags, when there is no
**                          more data to be sent. Otherwise IRQ will newer change state.
*********************************************************************************************************/
void clearBufferTransmitIfFlags(CanbusConfig_t* canCfg,uint8_t flags) {
    flags &= MCP_TX_INT;
    if (flags == 0) {
        return;
    }
    mcp2515_modifyRegister(canCfg,MCP_CANINTF, flags, 0);
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            check if got something
*********************************************************************************************************/
uint8_t checkReceive(CanbusConfig_t* canCfg)
{
    uint8_t res;
    res = mcp2515_readStatus(canCfg);
    // RXnIF in Bit 1 and 0
    return ((res & MCP_STAT_RXIF_MASK) ? CAN_MSGAVAIL : CAN_NOMSG);
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            if something error
*********************************************************************************************************/
uint8_t checkError(CanbusConfig_t* canCfg,uint8_t* err_ptr)
 {
    uint8_t eflg = mcp2515_readRegister(canCfg,MCP_EFLG);

    if (err_ptr)
	{
        *err_ptr = eflg;
    }
    return ((eflg & MCP_EFLG_ERRORMASK) ? CAN_CTRLERROR : CAN_OK);
}

/*********************************************************************************************************
** Function name:           mcpPinMode
** Descriptions:            switch supported pins between HiZ, interrupt, output or input
*********************************************************************************************************/
int8_t mcpPinMode(CanbusConfig_t* canCfg,const uint8_t pin, const uint8_t mode)
{
    uint8_t res;
    int8_t ret = true;

    switch (pin)
	{
        case MCP_RX0BF:
            switch (mode)
			{
                case MCP_PIN_HIZ:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B0BFE, 0);
                    break;
                case MCP_PIN_INT:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B0BFM | B0BFE, B0BFM | B0BFE);
                    break;
                case MCP_PIN_OUT:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B0BFM | B0BFE, B0BFE);
                    break;
                default:
                    #if DEBUG_EN
                    DBG_PRINT("Invalid pin mode request");
                    #endif
                    return false;
            }
            return true;
            break;
        case MCP_RX1BF:
            switch (mode)
			{
                case MCP_PIN_HIZ:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B1BFE, 0);
                    break;
                case MCP_PIN_INT:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B1BFM | B1BFE, B1BFM | B1BFE);
                    break;
                case MCP_PIN_OUT:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B1BFM | B1BFE, B1BFE);
                    break;
                default:
                    #if DEBUG_EN
                    DBG_PRINT("Invalid pin mode request");
                    #endif
                    return false;
            }
            return true;
            break;
        case MCP_TX0RTS:

            res = mcp2515_setCANCTRL_Mode(canCfg,MODE_CONFIG);

            if (res > 0)
			{
                #if DEBUG_EN
                DBG_PRINT("Entering Configuration Mode Failure...");
                #else
                HW_DELAY(10);
                #endif
                return false;
            }
            switch (mode)
			{
                case MCP_PIN_INT:
                    mcp2515_modifyRegister(canCfg,MCP_TXRTSCTRL, B0RTSM, B0RTSM);
                    break;
                case MCP_PIN_IN:
                    mcp2515_modifyRegister(canCfg,MCP_TXRTSCTRL, B0RTSM, 0);
                    break;
                default:
                    #if DEBUG_EN
                    DBG_PRINT("Invalid pin mode request");
                    #endif
                    ret = false;
            }
            res = mcp2515_setCANCTRL_Mode(canCfg,canCfg->mcpMode);
            if (res)
			{
                #if DEBUG_EN
                DBG_PRINT("`Setting ID Mode Failure...");
                #else
                HW_DELAY(10);
                #endif
                return false;
            }
            return ret;
            break;
        case MCP_TX1RTS:
            res = mcp2515_setCANCTRL_Mode(canCfg,MODE_CONFIG);
            if (res > 0)
			{
                #if DEBUG_EN
                DBG_PRINT("Entering Configuration Mode Failure...");
                #else
                HW_DELAY(10);
                #endif
                return false;
            }
            switch (mode)
			{
                case MCP_PIN_INT:
                    mcp2515_modifyRegister(canCfg,MCP_TXRTSCTRL, B1RTSM, B1RTSM);
                    break;
                case MCP_PIN_IN:
                    mcp2515_modifyRegister(canCfg,MCP_TXRTSCTRL, B1RTSM, 0);
                    break;
                default:
                    #if DEBUG_EN
                    DBG_PRINT("Invalid pin mode request");
                    #endif
                    ret = false;
            }
            res = mcp2515_setCANCTRL_Mode(canCfg,canCfg->mcpMode);
            if (res)
			{
                #if DEBUG_EN
                DBG_PRINT("`Setting ID Mode Failure...");
                #else
                HW_DELAY(10);
                #endif
                return false;
            }
            return ret;
            break;
        case MCP_TX2RTS:
            res = mcp2515_setCANCTRL_Mode(canCfg,MODE_CONFIG);

            if (res > 0)
			{
                #if DEBUG_EN
                DBG_PRINT("Entering Configuration Mode Failure...");
                #else
                HW_DELAY(10);
                #endif
                return false;
            }
            switch (mode)
			{
                case MCP_PIN_INT:
                    mcp2515_modifyRegister(canCfg,MCP_TXRTSCTRL, B2RTSM, B2RTSM);
                    break;
                case MCP_PIN_IN:
                    mcp2515_modifyRegister(canCfg,MCP_TXRTSCTRL, B2RTSM, 0);
                    break;
                default:
                    #if DEBUG_EN
                    DBG_PRINT("Invalid pin mode request");
                    #endif
                    ret = false;
            }
            res = mcp2515_setCANCTRL_Mode(canCfg,canCfg->mcpMode);
            if (res)
			{
                #if DEBUG_EN
                DBG_PRINT("`Setting ID Mode Failure...");
                #else
                HW_DELAY(10);
                #endif
                return false;
            }
            return ret;
            break;
        default:
            #if DEBUG_EN
            DBG_PRINT("Invalid pin for mode request");
            #endif
            return false;
    }
}

/*********************************************************************************************************
** Function name:           mcpDigitalWrite
** Descriptions:            write HIGH or LOW to RX0BF/RX1BF
*********************************************************************************************************/
int8_t mcpDigitalWrite(CanbusConfig_t* canCfg,const uint8_t pin, const uint8_t mode)
{
    switch (pin)
	{
        case MCP_RX0BF:
            switch (mode)
			{
                case HIGH:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B0BFS, B0BFS);
                    return true;
                    break;
                default:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B0BFS, 0);
                    return true;
            }
            break;
        case MCP_RX1BF:
            switch (mode)
			{
                case HIGH:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B1BFS, B1BFS);
                    return true;
                    break;
                default:
                    mcp2515_modifyRegister(canCfg,MCP_BFPCTRL, B1BFS, 0);
                    return true;
            }
            break;
        default:
            #if DEBUG_EN
            DBG_PRINT("Invalid pin for mcpDigitalWrite");
            #endif
            break;
    }

	return false;
}

/*********************************************************************************************************
** Function name:           mcpDigitalRead
** Descriptions:            read HIGH or LOW from supported pins
*********************************************************************************************************/
uint8_t mcpDigitalRead(CanbusConfig_t* canCfg,const uint8_t pin)
{
    switch (pin)
	{
        case MCP_RX0BF:
            if ((mcp2515_readRegister(canCfg,MCP_BFPCTRL) & B0BFS) > 0)
			{
                return HIGH;
            }
			else
			{
                return LOW;
            }
            break;
        case MCP_RX1BF:
            if ((mcp2515_readRegister(canCfg,MCP_BFPCTRL) & B1BFS) > 0)
			{
                return HIGH;
            }
			else
			{
                return LOW;
            }
            break;
        case MCP_TX0RTS:
            if ((mcp2515_readRegister(canCfg,MCP_TXRTSCTRL) & B0RTS) > 0)
			{
                return HIGH;
            }
			else
			{
                return LOW;
            }
            break;
        case MCP_TX1RTS:
            if ((mcp2515_readRegister(canCfg,MCP_TXRTSCTRL) & B1RTS) > 0)
			{
                return HIGH;
            }
			else
			{
                return LOW;
            }
            break;
        case MCP_TX2RTS:
            if ((mcp2515_readRegister(canCfg,MCP_TXRTSCTRL) & B2RTS) > 0)
			{
                return HIGH;
            }
			else
			{
                return LOW;
            }
            break;
        default:
            #if DEBUG_EN
            DBG_PRINT("Invalid pin for mcpDigitalRead");
            #endif
            return LOW;
    }
}

/*********************************************************************************************************
    END FILE
*********************************************************************************************************/







