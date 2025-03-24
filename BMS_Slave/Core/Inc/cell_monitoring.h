/*
 * cell_monitoring.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Alexis G.
 */

#ifndef CELL_MONIT_H_
#define CELL_MONIT_H_

//#include "can_bus.h"
#include "stm32g0xx_hal.h"

// Comm with BQ chip
extern I2C_HandleTypeDef hi2c2;

// BQ address on I2C
#define BQ76940_ADDR 0x08

// Voltage
#define VC1_LO_ADDR 0x0D
#define NB_CELLS 15

// Temps
#define TS1_HI_ADDR 0x2C
#define NB_TEMP_SENS 3

// Public knowledge
extern uint16_t cell_voltages[NB_CELLS];
extern uint16_t pack_temps[NB_TEMP_SENS];

void Task_cell_motoring();

uint16_t ReadPackTemp(uint8_t sens_reg_LSB);
uint16_t ReadCellVoltage(uint8_t cell_reg_LSB);

HAL_StatusTypeDef BQ76940_WriteRegister(uint8_t reg, uint8_t data);
HAL_StatusTypeDef BQ76940_ReadRegister(uint8_t reg, uint8_t *data);


#endif /* CELL_MONIT_H_ */
