/*
 * TMC-SPI.h: Definitions for TMC SPI wrapper interface
 *
 *  Created on: Dec 11, 2019
 *      Author: tbgiles
 */

#ifndef SPI_TMC_H
#define SPI_TMC_H

#include "stm32f4xx_hal.h"
#include "../TMC-API/tmc/ic/TMC5160/TMC5160.h"
#include "motioncontrol.h"


// TMC5160 SPI wrapper
void tmc5160_writeDatagram(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
void tmc5160_writeInt(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address, uint32_t value);
uint32_t tmc5160_readInt(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address);

// General SPI functions
void tmc40bit_writeInt(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address, uint32_t value);
uint32_t tmc40bit_readInt(SPI_HandleTypeDef* hspi, e_motor motor, uint8_t address);

void AD_TMC5160_Init();

#endif /* SPI_TMC_H */
