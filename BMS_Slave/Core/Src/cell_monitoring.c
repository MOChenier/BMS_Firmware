/*
 * cell_monitoring.c
 *
 *  Created on: Mar 22, 2025
 *      Author: Alexis G.
 */


#include "cell_monitoring.h"

uint16_t cell_voltages[NB_CELLS];
uint16_t pack_temps[NB_TEMP_SENS];


void Task_cell_motoring(){


	for (int i=0; i<NB_CELLS; i=+2) {
		cell_voltages[i] = ReadCellVoltage(i);
	}

	for (int i=0; i<NB_TEMP_SENS; i++) {
		pack_temps[i] = ReadPackTemp(i);
	}

	// Send voltages on CAN Network
	//FDCAN_sendMessages()
}

// Read one cell's voltage
uint16_t ReadPackTemp(uint8_t sens_reg_LSB) {
    uint8_t lsb = 0, msb = 0;
    BQ76940_ReadRegister(TS1_HI_ADDR, &lsb);
    BQ76940_ReadRegister(TS1_HI_ADDR + 1, &msb);
    return ((uint16_t)msb << 8) | lsb;
}

// Read one cell's voltage
uint16_t ReadCellVoltage(uint8_t cell_reg_LSB) {
    uint8_t lsb = 0, msb = 0;
    BQ76940_ReadRegister(cell_reg_LSB, &lsb);
    BQ76940_ReadRegister(cell_reg_LSB + 1, &msb);
    return ((uint16_t)msb << 8) | lsb;
}

// I2C Write to the BQ76940
HAL_StatusTypeDef BQ76940_WriteRegister(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return HAL_I2C_Master_Transmit(&hi2c2, BQ76940_ADDR, buf, 2, HAL_MAX_DELAY);
}

// I2C Read from the BQ76940
HAL_StatusTypeDef BQ76940_ReadRegister(uint8_t reg, uint8_t *data) {
    HAL_StatusTypeDef status;
    status = HAL_I2C_Master_Transmit(&hi2c2, BQ76940_ADDR, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return status;
    return HAL_I2C_Master_Receive(&hi2c2, BQ76940_ADDR, data, 1, HAL_MAX_DELAY);
}

