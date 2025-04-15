#include <slave_com.h>

extern CAN_HandleTypeDef hcan2;

extern return_mess_t voltage_info_list[];
extern return_mess_t temperature_info_list[];
extern return_mess_t balancing_info_list[];


void init_slave_info_lists()
{
	for(int i = 0; i < NUM_OF_SLAVES; i++){
		return_mess_t last_voltage_info = {{0}, 0, 0};
		return_mess_t last_temp_info = {{0}, 0, 0};
		return_mess_t last_balanc_info = {{0}, 0, 0};

		voltage_info_list[i] = last_voltage_info;
		temperature_info_list[i] = last_temp_info;
		balancing_info_list[i] = last_balanc_info;
	}
}


int store_received_info(CAN_RxHeaderTypeDef* RxHeader, uint8_t RxData[8]){

	if((RxHeader->StdId & INFO_ID_MASK ) == RET_VOLTAGE_INFO_STDID){


		for(int i = 0; i < 8; i++){

			voltage_info_list[RxHeader->StdId & SLAVE_ID_MASK].RxData[i] = RxData[i];

		}
		return 1;
	}else if((RxHeader->StdId & INFO_ID_MASK ) == RET_TEMP_INFO_STDID){

		for(int i = 0; i < 8; i++){

			temperature_info_list[RxHeader->StdId & SLAVE_ID_MASK].RxData[i] = RxData[i];

		}
		return 2;

	}else if((RxHeader->StdId & INFO_ID_MASK ) == RET_BALANC_INFO_STDID){

		for(int i = 0; i < 8; i++){

			balancing_info_list[RxHeader->StdId & SLAVE_ID_MASK].RxData[i] = RxData[i];

		}
		return 3;
	}


	return 0;
}

int all_voltages_under_threshold()
{
	//Check that cells are all under max voltage
	uint8_t checksum = 0;

	for(int i = 0; i < NUM_OF_SLAVES; i++){

		checksum = voltage_info_list[i].RxData[0] | voltage_info_list[i].RxData[1];

	//	If one cell is at or higher than max allowed voltage, return 0 (false)
		if (checksum != 0)
			return 0;
	}

	return 1;
}


float get_pack_voltage(uint8_t pack_id)
{

	return voltage_info_list[pack_id].RxData[5];

}

int all_balancing_charge_present(){

	//Check that cells are all under max temperature
	for(int i = 0; i< NUM_OF_SLAVES; i++){

		if(temperature_info_list[i].RxData[5] == 0)
				return 0;

	}
	return 1;
}

int all_temps_under_threshold()
{
	//Check that cells are all under max temperature
	for(int i = 0; i< NUM_OF_SLAVES; i++){

		for(int j = 0; j < 8; j++){

			if(temperature_info_list[i].RxData[j] >= TEMP_THRESH)
				return 0;
		}
	}

	return 1;
}

int all_cells_balanced()
{
	uint8_t checksum;

	//Check that cells are all balanced
	for(int i = 0; i < NUM_OF_SLAVES;  i++){

		checksum = balancing_info_list[i].RxData[0] | balancing_info_list[i].RxData[1];

		//	If one cell is unbalanced, return 0 (false)
		if(checksum != 0)
			return 0;
	}
	return 1;
}

void ask_for_voltages()
{

	uint8_t message[2] = {0};

	message[0] = VOLTAGE_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void auto_ask_for_voltages(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_VOLTAGE_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void stop_auto_ask_for_voltages()
{

	uint8_t message[2] = {0};

	message[0] = STOP_VOLTAGE_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void ask_for_temperatures()
{

	uint8_t message[2] = {0};

	message[0] = TEMP_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void auto_ask_for_temperatures(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_TEMP_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void stop_auto_ask_for_temperatures()
{

	uint8_t message[2] = {0};

	message[0] = STOP_TEMP_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void ask_for_balancing_info()
{

	uint8_t message[2] = {0};

	message[0] = BALANCING_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void auto_ask_for_balancing_info(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_BALANCING_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void stop_auto_ask_for_balancing_info()
{

	uint8_t message[2] = {0};

	message[0] = STOP_BALANCING_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void start_balancing()

{
	uint8_t message[2] = {0};

	message[0] = START_BALANCING;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void stop_balancing()

{
	uint8_t message[2] = {0};

	message[0] = STOP_BALANCING;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}

void get_ready_for_charging()
{
	uint8_t message[2] = {0};

	message[0] = READY_TO_CHARGE;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN_send_mess(&hcan2, message);

}



