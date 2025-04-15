/*
 * charger.c
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */


#include "charger.h"


extern CAN_HandleTypeDef hcan2;
extern uint8_t master_mode;

extern volatile uint8_t  all_connection_states;


void charging_balancing_mode(){

	uint8_t skip = 1; //Initialize quit to 1 so it runs skips charging on first iteration


	master_mode = CHARGING_BALANCING_MODE;

//	Ask for a single readout of cell voltages
	ask_for_voltages();

	HAL_Delay(100);

//	If all cells are under max voltage, start with charging the battery pack
	if(all_voltages_under_threshold()){
		skip = 0;
	}

	// 3 different exit possibilities : Charger disconnected, No balancing charge, Balancing done
	while(1)
	{

		if(skip)
		{
			skip = 0; // Once we skipped one time we run it normally

		} else{

//			Start auto voltage and temperature send. Start charging
			start_charging_mode();

			// While charger is connected. Can also exit if cells are at max threshold for temperature or voltage.
			while((all_connection_states & CHARGER_CONN_MASK) != 0){

//				Sand message to charge every loop. If the charger doesn't receive message for more than 5 seconds it cuts off
				charger_charging_active();

				if(!all_voltages_under_threshold())
					break;

				if(!all_temps_under_threshold())
					break;


				HAL_Delay(CHARGING_LOOP_DELAY);
			}

			stop_charging_mode();
		}


		HAL_Delay(CHARGING_STOP_DELAY); //Delay to make sure that charging has stopped

		// If charger is disconnected, skip balancing to maintain battery level
		if((all_connection_states & CHARGER_CONN_MASK) == 0)
			break;

//		Start auto send voltage and balancing info. Start balancing
		start_balancing_mode();

//		Exits if balancing is done or if balancing charge is disconnected.
		while(1)
		{
			// If balancing charge is not present, quit.
			if(!all_balancing_charge_present())
				break;

			// If all cells are balanced, quit.
			if(all_cells_balanced())
				break;


			HAL_Delay(BALANCING_LOOP_DELAY);

		}

		stop_balancing_mode();

		HAL_Delay(1000);

//		If battery full and balanced, exits balancing/charging mode. Else, do another round of charging/balancing.
		if(battery_full_and_balanced())
			break;

	}

}


void start_charging_mode()
{
	auto_ask_for_temperatures(CHARGING_TEMP_INFO_DELAY); // 10 ms delay between each measure
	auto_ask_for_voltages(CHARGING_VOLT_INFO_DELAY);   // 10 ms delay between each measure
	charger_charging_active();
}

void stop_charging_mode()
{
	stop_auto_ask_for_temperatures();
	stop_auto_ask_for_voltages();
	stop_charging();
}

void start_balancing_mode()
{
	auto_ask_for_balancing_info(BALANCING_BAL_INFO_DELAY);
	auto_ask_for_temperatures(BALANCING_TEMP_INFO_DELAY);
	start_balancing();
}

void stop_balancing_mode()
{
	stop_auto_ask_for_balancing_info();
	stop_auto_ask_for_temperatures();
	stop_balancing();
}

void charger_charging_active()
// Message must be sent at least every second. Otherwise charger cuts power.
{
	uint8_t message[8] = {0};

	uint16_t max_voltage = 0;
	uint16_t max_current = 0;


	max_voltage = (uint16_t)(CHARGER_MAX_VOLTAGE * 10);
	max_current = (uint16_t)(CHARGER_MAX_CURRENT * 10);

//	Max voltage value high byte then low byte
	message[0] = (uint8_t)(max_voltage && 0xFF00) >> 8;
	message[1] = (uint8_t)(max_voltage && 0x00FF);

//	Max current value high byte then low byte
	message[2] = (uint8_t)(max_current && 0xFF00) >> 8;
	message[3] = (uint8_t)(max_current && 0x00FF);

//	Start charging
	message[4] = 0;

	CAN_set_ext_header(CHARGER_TX_HEADER, CHARGER_MESS_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void stop_charging()
{
	uint8_t message[8] = {0};

	uint16_t max_voltage = 0;
	uint16_t max_current = 0;


	max_voltage = (uint16_t)(CHARGER_MAX_VOLTAGE * 10);
	max_current = (uint16_t)(CHARGER_MAX_CURRENT * 10);

//	Max voltage value high byte then low byte
	message[0] = (uint8_t)(max_voltage && 0xFF00) >> 8;
	message[1] = (uint8_t)(max_voltage && 0x00FF);

//	Max current value high byte then low byte
	message[2] = (uint8_t)(max_current && 0xFF00) >> 8;
	message[3] = (uint8_t)(max_current && 0x00FF);

//	Stop charging
	message[4] = 1;

	CAN_set_ext_header(CHARGER_TX_HEADER, CHARGER_MESS_BYTES_NUM);
	CAN_send_mess(&hcan2, message);

}



int battery_full_and_balanced()
{
	uint8_t is_not_done = 0;
	ask_for_voltages();
	ask_for_balancing_info();

	HAL_Delay(1000);

	//check if all cells are balanced and near total pack target voltage
	//update b2 of 'charging_balancing_states' if these two conditions are met

	for(int i = 0; i < NUM_OF_SLAVES; i++){

		// Calculates state of charge of pack. If it is under target value, mark it as not done.
		if((get_pack_voltage(i) - MIN_BATT_VOLTAGE)/(MAX_BATT_VOLTAGE - MIN_BATT_VOLTAGE) < TARGET_BATT_CHARG_STATE)
			is_not_done++;

	}

	// If not all cells are balanced, mark it as not done.
	if(!all_cells_balanced())
		is_not_done++;

	// Update balancing done bit
	if(is_not_done == 0)
		return 1;

	return 0;
}





