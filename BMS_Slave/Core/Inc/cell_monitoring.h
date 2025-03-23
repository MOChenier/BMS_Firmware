/*
 * cell_monitoring.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Alexis G.
 */

#ifndef CELL_MONIT_H_
#define CELL_MONIT_H_

#include "can_bus.h"
#include "stm32g0xx_hal.h"


//#define CHARGING_LOOP_DELAY 10

//#define CELLS_UNDER_THRESH_MASK 0x01
//#define BALANCING_DONE_MASK 0x02
//#define BALANCING_CHARGE_PRESENT_MASK 0x04

void charging_balancing_mode();


#endif /* CELL_MONIT_H_ */
