/*
 * mcp2515_hardware.c
 *
 *  Created on: Nov 25, 2022
 *      TORK ROBOTIK
 */

#include "mcp2515_hardware.h"





void ChipUnSelectFpC(void)
{
	HAL_GPIO_WritePin(CS_CAN_C_GPIO_Port, CS_CAN_C_Pin, GPIO_PIN_SET);
}

void ChipSelectFpC(void)
{
	HAL_GPIO_WritePin(CS_CAN_C_GPIO_Port, CS_CAN_C_Pin, GPIO_PIN_RESET);
}

void SPIReadWriteFpC(uint8_t data)
{
	 HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, 100);
}

void SPIWriteFpC(uint8_t data)
{
	 HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, 100);
}

uint8_t SPIReadFpC(void)
{
	uint8_t data = 0x00;

	HAL_SPI_Receive(&hspi1, (uint8_t *)&data, 1, 100);

	return data;
}



void delayMicroseconds(uint32_t microseconds)
{
    uint32_t loops_per_microsecond = 72;
    uint32_t total_loops = microseconds * loops_per_microsecond;

    // Döngü ile bekle
    for (uint32_t i = 0; i < total_loops; ++i) {

    }
}
