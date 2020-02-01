/*
 * TMC-SPI.c
 *
 *  Created on: Dec 11, 2019
 *      Author: tbgiles
 */

#include "TMC-SPI.h"
#include "stm32f4xx_hal.h"



// TMC5160 SPI wrapper
void tmc5160_writeDatagram(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
    uint32_t value = x1;
	value <<= 8;
	value |= x2;
	value <<= 8;
	value |= x3;
	value <<= 8;
	value |= x4;

    tmc40bit_writeInt(hspi, motor, address, value);
}

void tmc5160_writeInt(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address, uint32_t value)
{
    tmc40bit_writeInt(hspi, motor, address, value);
}

uint32_t tmc5160_readInt(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address)
{
    tmc40bit_readInt(hspi, motor, address);
    return tmc40bit_readInt(hspi, motor, address);
}

// General SPI decription
void tmc40bit_writeInt(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address, uint32_t value)
{
    uint8_t tbuf[5];
    tbuf[0] = address | 0x80;
    tbuf[1] = 0xFF & (value>>24);
    tbuf[2] = 0xFF & (value>>16);
    tbuf[3] = 0xFF & (value>>8);
    tbuf[4] = 0xFF & value;

    // Blindly transmit
    HAL_SPI_Transmit(hspi, tbuf, 5, HAL_MAX_DELAY);
    // OLD bcm2835_spi_writenb (tbuf, 5);
}

uint32_t tmc40bit_readInt(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address)
{
    uint8_t tbuf[5], rbuf[5];
    uint32_t value;
	// clear write bit, send dummy data
	tbuf[0] = address & 0x7F;

	HAL_SPI_TransmitReceive(hspi, tbuf, rbuf, 5, HAL_MAX_DELAY);
    //bcm2835_spi_transfernb (tbuf, rbuf, 5);

	value =rbuf[1];
	value <<= 8;
	value |= rbuf[2];
	value <<= 8;
	value |= rbuf[3];
	value <<= 8;
	value |= rbuf[4];

	return value;
}



