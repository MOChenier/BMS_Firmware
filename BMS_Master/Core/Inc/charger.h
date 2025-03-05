/*
 * charger.h
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */

#ifndef INC_CHARGER_H_
#define INC_CHARGER_H_

#include "can_bus.h"
#include "stm32f4xx_hal.h"


#define CHARGING_LOOP_DELAY 10

#define CELLS_UNDER_THRESH_MASK 0x01
#define BALANCING_DONE_MASK 0x02
#define BALANCING_CHARGE_PRESENT_MASK 0x04

void charging_balancing_mode();


#endif /* INC_CHARGER_H_ */
