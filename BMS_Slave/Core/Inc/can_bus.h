/*
 * can_bus.h
 *
 *  Created on: Avril 11, 2025
 *      Author: Alexis (aka: ChatGPT + le code de Marco)
 */

#ifndef INC_CAN_BUS_H_
#define INC_CAN_BUS_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "stm32g0xx_hal.h"  // Replace with your MCU's HAL if not F4
#include "main.h"

#define TxID (0x111)
#define TxDLC 2  // For FDCAN, max 64 bytes in FD mode

void FDCAN_init();
void FDCAN_set_std_header(uint16_t std_tx_id, uint8_t dlc);
void FDCAN_set_ext_header(uint32_t ext_tx_id, uint8_t dlc);

int FDCAN1_send_mess(FDCAN_HandleTypeDef *hfdcan, uint8_t *TxData);
void FDCAN_Error_Handler(void);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void FDCAN_Filter_Config(FDCAN_HandleTypeDef *hfdcan);
void FDCAN_Activate_Interrupts(FDCAN_HandleTypeDef *hfdcan);
void MX_FDCAN1_Init_Loopback(FDCAN_HandleTypeDef *hfdcan);

typedef struct slave_return {
	uint8_t		content_header;		// Indicates what type of cell state is in the message
	uint16_t	cell_states;		// One bit per cell. Indicates if the state is normal (0) or problematic (1)
	uint8_t		min_value_raw;		// Minimum value of the cells
	uint8_t		max_value_raw;		// Maximum value of the cells
	uint8_t		avg_value_raw;		// Average value of cells
} slave_return_t;

#endif /* INC_CAN_BUS_H_ */
