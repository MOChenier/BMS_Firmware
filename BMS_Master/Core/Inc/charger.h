/*
 * charger.h
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */

#ifndef INC_CHARGER_H_
#define INC_CHARGER_H_

#include <slave_com.h>
#include "can_bus.h"
#include "stm32f4xx_hal.h"
#include "misc.h"

#define CHARGING_LOOP_DELAY 500
#define BALANCING_LOOP_DELAY 10
#define CHARGING_STOP_DELAY 5000

#define CELLS_UNDER_THRESH_MASK 0x01
#define BALANCING_DONE_MASK 0x02
#define BALANCING_CHARGE_PRESENT_MASK 0x04

#define CHARGING_TEMP_INFO_DELAY	100
#define CHARGING_VOLT_INFO_DELAY	100

#define BALANCING_TEMP_INFO_DELAY	100
#define BALANCING_VOLT_INFO_DELAY	100
#define BALANCING_BAL_INFO_DELAY	100


// Info for charger CAN communication
#define CHARGER_MESS_BYTES_NUM 8

#define CHARGER_HEAD_PRIO 	8   // Message priority
#define CHARGER_HEAD_R		0  	// R is generally 0 according to doc
#define CHARGER_HEAD_DP		0  	// DP is fixed at 0 according to doc
#define CHARGER_HEAD_PF		0xFF // PF is the code for the message ?
#define CHARGER_HEAD_PS		0xE5 // Destination Address (charger control system)
#define CHARGER_HEAD_SA		0xF4 // Source address (BMS)

#define CHARGER_MAX_VOLTAGE 126
#define CHARGER_MAX_CURRENT 46


#define CHARGER_TX_HEADER		(uint32_t)((CHARGER_HEAD_SA 	&& 0xFF) \
									& ((CHARGER_HEAD_PS 	&& 0xFF) << 8) \
									& ((CHARGER_HEAD_PF 	&& 0xFF) << 16) \
									& ((CHARGER_HEAD_DP 	&& 0x01) << 24) \
									& ((CHARGER_HEAD_R 		&& 0x01) << 25) \
									& ((CHARGER_HEAD_PRIO 	&& 0x07) << 26))


void charging_balancing_mode();
void start_charging_mode();
void stop_charging_mode();
void charger_charging_active();
void stop_charging();
void start_balancing_mode();
void stop_balancing_mode();
int all_cells_under_threshold();
int battery_full_and_balanced();



#endif /* INC_CHARGER_H_ */
