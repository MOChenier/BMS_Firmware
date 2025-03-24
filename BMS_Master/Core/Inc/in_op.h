/*
 * ignition.h
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */

#ifndef INC_IN_OP_H_
#define INC_IN_OP_H_

#include "stm32f4xx_hal.h"
#include "misc.h"
#include "main.h"




#define PRECHARGE_DELAY_S	15
#define PRECHARGE_DELAY_MS	PRECHARGE_DELAY_S*1000

#define OVERLAP_DELAY_S		1
#define OVERLAP_DELAY_MS	OVERLAP_DELAY_S*1000

#define MAX_CURRENT_THRESHOLD (float)(200)

#define IGNITION_LOOP_DELAY_MS 10



int check_if_ignition_ON(void);
void in_operation_mode(void);


#endif /* INC_IN_OP_H_ */
