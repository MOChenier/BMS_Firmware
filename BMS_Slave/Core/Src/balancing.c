/*
 * balancing.c
 *
 *  Created on: Mar 22, 2025
 *      Author: Alexis G.
 */


#include "balancing.h"
#include "cell_monitoring.h"

uint16_t delta_v;

void Task_balancing(){

	// sort voltages
    for (int i = 0; i < NB_CELLS - 1; i++) {
        int max_idx = i;
        for (int j = i + 1; j < NB_CELLS; j++) {
            if (cell_voltages[j] > cell_voltages[max_idx]) {
                max_idx = j;
            }
        }
        // Swap arr[i] and arr[max_idx]
        int temp = arr[i];
        cell_voltages[i] = cell_voltages[max_idx];
        cell_voltages[max_idx] = temp;
    }

	// calculate delta_V
    delta_v = cell_voltages[NB_CELLS-1] - cell_voltages[0];

	// Pick cells to balance


	// Check if charges are connected
    // Flashes yellow light if not
    while(HAL_GPIO_ReadPin(GPIOB, 6) == LOW){
    	HAL_GPIO_TogglePin(GPIOA, 15);
    	HAL_Delay(100);
    }

}

void set_balancing(uint16_t res_states){

	for(int i=0; i<NB_CELLS; i++){
		BQ76940_WriteRegister(reg, data)
	}

}


