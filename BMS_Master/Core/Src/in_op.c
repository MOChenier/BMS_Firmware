/*
 * ignition.c
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */

#include <in_op.h>

//	Flag modified on message reception if a cell is out of the operating range (temperature or voltage)
uint8_t cell_out_of_op_range = 0;

extern volatile uint8_t all_connection_states;
extern uint8_t master_mode;

void in_operation_mode(void)
{
	master_mode = IN_OPERATION_MODE;

	if((all_connection_states & EMERGENCY_STOP_MASK) == 0)
		activate_precharge_contactor();

//	Wait for the circuit to be precharged
	HAL_Delay(PRECHARGE_DELAY_MS);

	if((all_connection_states & EMERGENCY_STOP_MASK) == 0){
		activate_main_contactor();

		HAL_Delay(OVERLAP_DELAY_MS);

		deactivate_precharge_contactor();
	}
//	Asks all slaves to send periodic readings of
//	turn_ON_slave_periodic_readings();

	while (1)
	{
//		Check if ignition is still ON
		if ((all_connection_states & IGNITION_MASK) == 0)
			break;

		if ((all_connection_states & EMERGENCY_STOP_MASK) != 0)
			break;

//		Validate that current output by the battery is under safety threshold
		if (get_output_current() >= MAX_CURRENT_THRESHOLD)
		{
			break;
		}

		if (cell_out_of_op_range)
		{
			break;
		}



		HAL_Delay(IGNITION_LOOP_DELAY_MS);
	}

	deactivate_main_contactor();
	deactivate_precharge_contactor();

}

