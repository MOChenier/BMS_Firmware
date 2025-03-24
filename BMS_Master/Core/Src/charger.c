/*
 * charger.c
 *
 *  Created on: Jan 16, 2025
 *      Author: marco
 */


#include "charger.h"


extern CAN_HandleTypeDef hcan2;

extern volatile uint8_t  all_connection_states;

uint8_t charging_balancing_states = 0; //b0: all cells at target value, b1: balancing done, b2: balancing charge not present,


void charging_balancing_mode(){

	uint8_t skip = 1; //Initialize quit to 1 so it runs skips charging on first iteration

	ask_for_voltages();

	HAL_Delay(100);

	if(all_cells_under_threshold()){
		skip = 0;
	}

	// 3 different exit possibilities : Charger disconnected, No balancing charge, Balancing done
	while(1){

		if(skip){
			start_charging_mode();

			// While charger is connected
			while((all_connection_states & CHARGER_CONN_MASK) != 0){

				//	adjust_charging();
				if(!all_cells_under_threshold())
					break;

				HAL_Delay(CHARGING_LOOP_DELAY);
			}

			stop_charging_mode();
		} else{

			skip = 0; // Once we skipped one time we run it normally
		}

		// If charger is disconnected, skip balancing to maintain battery level
		if((all_connection_states & CHARGER_CONN_MASK) == 0)
			break;


		start_balancing_mode();

		while(1){

			// If all cells at target value or balancing done or balancing charge not present
			if(charging_balancing_states != 0)
				break;

			HAL_Delay(BALANCING_LOOP_DELAY);

		}

		stop_balancing_mode();


		check_total_battery_state();

		// If balancing done, quit charging/balancing mode
		if((charging_balancing_states & BALANCING_DONE_MASK) != 0)
			break;

		// If balancing charge not present, quit charging/balancing mode
		if((charging_balancing_states & BALANCING_CHARGE_PRESENT_MASK) != 0)
			break;

	}

}


void start_charging_mode()
{
	auto_ask_for_temperatures(CHARGING_TEMP_INFO_DELAY); // 10 ms delay between each measure
	auto_ask_for_voltages(CHARGING_VOLT_INFO_DELAY);   // 10 ms delay between each measure
	start_charging();
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

//	Reset flags
	charging_balancing_states = 0;
}

void start_charging()
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

	CAN1_send_mess(&hcan2, message);

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
	CAN1_send_mess(&hcan2, message);

}


int all_cells_under_threshold()
{
	//Check that cells are all under max voltage and temperature
	return 0;
}

void check_total_battery_state()
{
	ask_for_voltages();
	HAL_Delay(100);

	//check if all cells are balanced and near total pack target voltage
	//update b2 of 'charging_balancing_states' if these two conditions are met

}



