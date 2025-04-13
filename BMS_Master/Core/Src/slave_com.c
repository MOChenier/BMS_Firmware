#include <slave_com.h>

extern CAN_HandleTypeDef hcan2;
extern uint8_t last_voltage_info[8];
extern uint8_t last_temp_info[8];
extern uint8_t last_balanc_info[8];

int store_received_info(CAN_RxHeaderTypeDef* RxHeader, uint8_t RxData[8]){

	if(RxHeader->StdId == RET_VOLTAGE_INFO_STDID){

		for(int i = 0; i < 8; i++){

			last_voltage_info[i] = RxData[i];

		}
		return 1;
	}else if(RxHeader->StdId == RET_TEMP_INFO_STDID){

		for(int i = 0; i < 8; i++){

			last_temp_info[i] = RxData[i];

		}
		return 2;

	}else if(RxHeader->StdId == RET_BALANC_INFO_STDID){

		for(int i = 0; i < 8; i++){

			last_balanc_info[i] = RxData[i];

		}
		return 3;
	}


	return 0;
}


void ask_for_voltages()
{

	uint8_t message[2] = {0};

	message[0] = VOLTAGE_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void auto_ask_for_voltages(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_VOLTAGE_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void stop_auto_ask_for_voltages()
{

	uint8_t message[2] = {0};

	message[0] = STOP_VOLTAGE_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void ask_for_temperatures()
{

	uint8_t message[2] = {0};

	message[0] = TEMP_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void auto_ask_for_temperatures(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_TEMP_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void stop_auto_ask_for_temperatures()
{

	uint8_t message[2] = {0};

	message[0] = STOP_TEMP_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void ask_for_balancing_info()
{

	uint8_t message[2] = {0};

	message[0] = BALANCING_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void auto_ask_for_balancing_info(uint8_t delay_10_ms)
{

	uint8_t message[2] = {0};

	message[0] = AUTO_BALANCING_INFO;
	message[1] = delay_10_ms;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void stop_auto_ask_for_balancing_info()
{

	uint8_t message[2] = {0};

	message[0] = STOP_BALANCING_INFO;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void start_balancing()

{
	uint8_t message[2] = {0};

	message[0] = START_BALANCING;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void stop_balancing()

{
	uint8_t message[2] = {0};

	message[0] = STOP_BALANCING;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}

void get_ready_for_charging()
{
	uint8_t message[2] = {0};

	message[0] = READY_TO_CHARGE;

	CAN_set_std_header(SLAVE_HEADER, SLAVE_BYTES_NUM);

	CAN1_send_mess(&hcan2, message);

}



