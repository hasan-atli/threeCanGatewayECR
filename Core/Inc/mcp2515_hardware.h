/*
 * mcp2515_hardware.h
 *
 *  Created on: Nov 25, 2022
 *      TORK ROBOTIK
 */

#ifndef INC_MCP2515_HARDWARE_H_
#define INC_MCP2515_HARDWARE_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"
#include "main.h"


extern SPI_HandleTypeDef hspi1;

/***************************************************/

void ChipUnSelectFpC(void);
void ChipSelectFpC(void);
void SPIReadWriteFpC(uint8_t data);
void SPIWriteFpC(uint8_t data);
uint8_t SPIReadFpC(void);

void delayMicroseconds(uint32_t microseconds);
/***************************************************/


#ifdef __cplusplus
}
#endif
#endif /* INC_MCP2515_HARDWARE_H_ */
