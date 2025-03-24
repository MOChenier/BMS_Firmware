/*
 * can_bus.h
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */

#ifndef INC_CAN_BUS_H_
#define INC_CAN_BUS_H_

#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <stdint.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */

#include "stm32f4xx_hal.h"
#include "main.h"

#define TxID (0x111)
#define TxDLC 2

void CAN_init();
void CAN_set_std_header(uint16_t std_tx_id, uint8_t dlc);
void CAN_set_ext_header(uint32_t ext_tx_id, uint8_t dlc);

int CAN1_send_mess(CAN_HandleTypeDef *hcan, uint8_t *TxData);
void CAN_Error_Handler(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void CAN_Filter_Config(CAN_HandleTypeDef *hcan);
void CAN_Activate_Interrupts(CAN_HandleTypeDef *hcan);
void MX_CAN1_Init_Loopback(CAN_HandleTypeDef *hcan);



typedef struct slave_return{

	uint8_t		content_header;		// Indicates what type of cell state is in the message

	uint16_t	cell_states;		// One bit per cell. Indicates if the state is normal (0) or problematic (1)

	uint8_t		min_value_raw;		// Minimum value of the cells
	uint8_t		max_value_raw;		// Maximum value of the cells
	uint8_t		avg_value_raw;		// Average value of cells

} slave_return_t;

#endif /* INC_CAN_BUS_H_ */
