/*
 * charger.c
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */


#include "charger.h"

extern volatile uint8_t  all_connection_states;

uint8_t charging_balancing_states = 0; //b0: all cells under threshold, b1: balancing done, b2: balancing charge present,


void charging_balancing_mode(){

	uint8_t quit = 1; //Initialize quit to 1 so it runs the loop only if all cells are under threshold

	ask_for_voltages();

	HAL_Delay(10);

	if(all_cells_under_treshold()){
		quit = 0;
	}

	while(quit == 0){

		start_charging();

		while((all_connection_states & CHARGER_CONN_MASK) == 1){
			ask_for_voltages();
			ask_for_temperatures();

			adjust_charging();

			if(!all_cells_under_treshold())
				break;

			HAL_Delay(CHARGING_LOOP_DELAY);
		}

		stop_charging();

		if((all_connection_states & CHARGER_CONN_MASK) == 0)
			break;

		start_balancing();

		while(1){

			ask_for_balancing_state();

			if(!charging_balancing_states)
				break;

			if(balancing_done())
				break;

		}

		if(!balancing_charge_present())
			break;

		stop_balancing();


	}

}
